/*
 * Mixed Reality and Applications
 *
 * Pedro Santana
 * 2013-2017 ISCTE-IUL
 *
 *  Assumed frames of reference:
 *  - PCL: z-blue (into the scene), x-red (to the right), y-green (downwards)
 *  - OSG: z (upwards), x (to the left), y (out of the scene)
 *
 */

#include <osg/Camera>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/GraphicsContext>
#include <osg/Depth>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/PositionAttitudeTransform>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osgGA/GUIEventHandler>
#include <osg/ShapeDrawable>
#include <osg/LineWidth>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <string>
#include <algorithm>
#include <sstream>

#include "../cloud_reader/models/cloudInitializer.h"

static const char *textureVertexSource = {
    "varying vec3 normal;\n"
    "void main()\n"
    "{\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"};

static const char *helicopterVertexSource = {
    "void main()\n"
    "{\n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0;\n"
    "    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * gl_Vertex;\n"
    "}\n"};

static const char *textureFramentSource = {
    "uniform sampler2D texture;\n"
    "uniform sampler2D textureDepth;\n"
    "void main()\n"
    "{\n"
    "   vec2 uv = vec2(gl_FragCoord.x/640.0, gl_FragCoord.y/480.0);\n"
    "   gl_FragColor = texture2D(texture, uv);\n"
    "   gl_FragDepth = (texture2D(textureDepth, uv)[2]);\n"
    "}\n"};

static const char *helicopterFragmentSource = {
    "uniform sampler2D texture;\n"
    "void main()\n"
    "{\n"
    "    gl_FragColor = texture2D(texture, gl_TexCoord[0].st);\n"
    "    gl_FragDepth = (1.0/gl_FragCoord.w)/1000.0;\n"
    "}\n"};

class HelicopterCallback : public osg::NodeCallback
{
public:
    HelicopterCallback(osg::PositionAttitudeTransform *_lineTransf) : _lineTransf(_lineTransf) {}
    virtual void operator()(osg::Node *node, osg::NodeVisitor *nv);

protected:
    osg::PositionAttitudeTransform *_lineTransf;
    osg::PositionAttitudeTransform *_circleTransf;
};

void HelicopterCallback::operator()(osg::Node *node, osg::NodeVisitor *nv)
{

    osg::PositionAttitudeTransform *helicopterTransf = static_cast<osg::PositionAttitudeTransform *>(node);
    osg::Vec3 v = helicopterTransf->getPosition();
    _lineTransf->setPosition(osg::Vec3(v.x(), v.y(), 0));
}

osg::Vec3Array *createCircle(int _radius, int _numberPoints)
{
    osg::Vec3Array *pts = new osg::Vec3Array();
    float angle = float(2.0 * osg::PI) / float(_numberPoints);
    for (unsigned int i = 0; i < _numberPoints; i++)
    {
        float x = _radius * cos(i * angle);
        float y = _radius * sin(i * angle);
        pts->push_back(osg::Vec3(x, y, 0));
    }
    return pts;
}

void CreateHelicopter(osg::ref_ptr<osg::PositionAttitudeTransform> helicopterTransf,
                      osg::ref_ptr<osg::PositionAttitudeTransform> lineTransf, osg::ref_ptr<osg::PositionAttitudeTransform> circleTransf)
{
    osg::ref_ptr<osg::Shader> vertShader2 =
        new osg::Shader(osg::Shader::VERTEX, helicopterVertexSource);
    osg::ref_ptr<osg::Shader> fragShader2 =
        new osg::Shader(osg::Shader::FRAGMENT, helicopterFragmentSource);
    osg::ref_ptr<osg::Program> program2 = new osg::Program;
    program2->addShader(fragShader2.get());
    program2->addShader(vertShader2.get());

    osg::Texture2D *helicopterTexture = new osg::Texture2D;
    helicopterTexture->setDataVariance(osg::Object::DYNAMIC);
    helicopterTexture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    helicopterTexture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    helicopterTexture->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP);
    helicopterTexture->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP);
    osg::Image *heliImage = osgDB::readImageFile("../Data/PlatonicSurface_Color.jpg");
    helicopterTexture->setImage(heliImage);

    osg::ref_ptr<osg::Node> helicopterNode = osgDB::readNodeFile("../Data/Helicopter.obj");
    helicopterNode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
    osg::StateSet *helicopterStateSet = helicopterNode->getOrCreateStateSet();
    helicopterStateSet->setTextureAttributeAndModes(0, helicopterTexture, osg::StateAttribute::ON);
    helicopterStateSet->setAttributeAndModes(program2.get());
    helicopterStateSet->addUniform(new osg::Uniform("texture", 0));

    //CIRCLE
    osg::Geometry *circle = new osg::Geometry();
    osg::Geode *circleGeode = new osg::Geode();

    osg::Vec4Array *colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0., 0., 0., 1.));
    circle->setColorArray(colors);
    circle->setVertexArray(createCircle(2, 20));
    circle->setColorBinding(osg::Geometry::BIND_OVERALL);
    circle->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP, 0, 20));
    circle->getOrCreateStateSet()->setAttributeAndModes(new osg::LineWidth(5), osg::StateAttribute::ON);

    circleGeode->addDrawable(circle);
    circleGeode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);

    circleTransf->addChild(circleGeode);
    circleTransf->setPosition(osg::Vec3(0, -100, 3));
    circleTransf->setDataVariance(osg::Object::DYNAMIC);
    circleTransf->setScale(osg::Vec3(0.02, 0.02, 0.02));

    helicopterTransf->addChild(helicopterNode);
    helicopterTransf->setScale(osg::Vec3(.04, .04, .04));
    helicopterTransf->setPosition(osg::Vec3(0, -100, 5));
    helicopterTransf->setAttitude(osg::Quat(0.0f, osg::Y_AXIS, 0.0f, osg::Z_AXIS, 0.0f, osg::X_AXIS));
    helicopterTransf->setDataVariance(osg::Object::DYNAMIC);

    // CYLINDER
    osg::Geode *cylinderGeode = new osg::Geode();
    osg::ShapeDrawable *cylinder = new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0, 0, 0), 0.1, 3));
    cylinder->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
    cylinder->setColor(osg::Vec4(0, 0, 0, 1));

    cylinderGeode->addDrawable(cylinder);
    cylinderGeode->setDataVariance(osg::Object::DYNAMIC);
    cylinderGeode->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
    osg::Material *pMaterial = new osg::Material;
    pMaterial->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
    cylinderGeode->getOrCreateStateSet()->setAttribute(pMaterial, osg::StateAttribute::OVERRIDE);

    lineTransf->addChild(cylinderGeode);
    lineTransf->setDataVariance(osg::Object::DYNAMIC);
    lineTransf->setPosition(osg::Vec3(0,-100, 2));
    lineTransf->setAttitude(osg::Quat(0.0f, osg::Y_AXIS, 0.0f, osg::Z_AXIS, 0.0f, osg::X_AXIS));
    lineTransf->setScale(osg::Vec3(10, 10, 10));

    static_cast<osg::Node *>(helicopterTransf)->setUpdateCallback(new HelicopterCallback(lineTransf.get()));
    //SET HERE THE CIRCLE THAT IS GOING TO FOLLOW THE HELICOPTER
}

class HelicopterController : public osgGA::GUIEventHandler
{
public:
    HelicopterController(osg::PositionAttitudeTransform *node, bool *_collidedLeft, bool *_collidedRight,
                         bool *_collidedFront, bool *_collidedBack, bool *_collidedBelow)
        : _helicopter(node), collidedLeft(_collidedLeft),
          collidedRight(_collidedRight), collidedFront(_collidedFront), collidedBack(_collidedBack), collidedBelow(_collidedBelow) {}
    virtual bool handle(const osgGA::GUIEventAdapter &ea,
                        osgGA::GUIActionAdapter &aa);

protected:
    osg::ref_ptr<osg::PositionAttitudeTransform> _helicopter;
    bool *collidedLeft, *collidedRight, *collidedFront, *collidedBack, *collidedBelow;
};

bool HelicopterController::handle(const osgGA::GUIEventAdapter &ea,
                                  osgGA::GUIActionAdapter &aa)
{
    if (!_helicopter)
        return false;

    //39.
    osg::Vec3 v = _helicopter->getPosition();

    switch (ea.getEventType())
    {

    case osgGA::GUIEventAdapter::KEYDOWN:
        switch (ea.getKey())
        {
        case 'a':
        case 'A':
            //40.
            if (!(*collidedLeft))
                v.x() += 1.0;
            else
                v.z() += 1.0;
            break;
        case 'd':
        case 'D':
            if (!(*collidedRight))
                v.x() -= 1.0;
            else
                v.z() += 1.0;
            break;
        case 'w':
        case 'W':
            //41.
            if (!(*collidedFront))
                v.y() -= 1.0;
            else
                v.z() += 1.0;
            break;
        case 's':
        case 'S':
            if (!(*collidedBack))
                v.y() += 1.0;
            else
                v.z() += 1.0;

            break;

        default:
            break;
        }

        if (!(*collidedBelow) && (!(*collidedBack) && !(*collidedFront) && !(*collidedRight) && !(*collidedLeft)))
            v.z() -= .5;

        //42.
        _helicopter->setPosition(v);

        break;
    default:
        break;
    }
    return false;
}

void estimateCameraPose(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, double *pitch, double *roll, double *height)
{

    //3.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_centre(new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_centre->height = 1;
    int step = 2;
    long index;

    for (int row = 0; row < cloud_in->height / 4; row += step)
    {

        for (int col = cloud_in->width / 2 - 50; col < cloud_in->width / 2 + 50; col += step)
        {
            index = (cloud_in->height - row - 1) * cloud_in->width + col;
            cloud_centre->points.push_back(cloud_in->points[index]);
            cloud_centre->width++;
        }
    }
    //4.
    pcl::PCDWriter writer;

    writer.write<pcl::PointXYZRGBA>("Out/cloud_centre.pcd", *cloud_centre, false);
    //5.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cloud_centre);
    seg.segment(*inliers, *coefficients);
    //6.
    double c_a = coefficients->values[0];
    double c_b = coefficients->values[1];
    double c_c = coefficients->values[2];
    double c_d = coefficients->values[3];
    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << "." << std::endl;

    //7.
    double norm = sqrt(c_a * c_a + c_b * c_b + c_c * c_c);
    c_a = c_a / norm;
    c_b = c_b / norm;
    c_c = c_c / norm;
    std::cout << "Coefficients a: " << c_a << " b: " << c_b << " c: " << c_c << " d: " << c_d << " norm: " << norm << std::endl;
    //8.A.
    if (c_c < 0)
    {
        c_a *= -1;
        c_b *= -1;
        c_c *= -1;
    }

    //8.
    *pitch = asin(c_c);
    *roll = -acos(c_b / cos(*pitch));
    *height = fabs(c_d);

    //8.B.
    if (c_a < 0)
        (*roll) *= -1;
    //9.
    std::cout << "Camera pitch: " << *pitch * 180 / M_PI << " [deg]; Camera roll: " << *roll * 180 / M_PI << " [deg]." << std::endl;

    std::cout << "Camera height: " << *height << std::endl;
}

void rotatePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated,
                      double camera_pitch, double camera_roll, double camera_height)
{

    //11.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_voxelised(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.setLeafSize(0.01, 0.01, 0.01);
    voxel_grid.filter(*cloud_voxelised);

    //12.
    Eigen::Affine3f t1 = pcl::getTransformation(0.0, -camera_height, 0.0, 0.0, 0.0, 0);
    Eigen::Affine3f t2 = pcl::getTransformation(0.0, 0.0, 0.0, -camera_pitch, 0.0, 0.0);
    Eigen::Affine3f t3 = pcl::getTransformation(0.0, 0.0, 0.0, 0.0, 0.0, -camera_roll);
    pcl::transformPointCloud(*cloud_voxelised, *cloud_rotated, t1 * t2 * t3);

    //13.
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGBA>("Out/out_rotated.pcd", *cloud_rotated, false);
}

void createImageFromPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, unsigned char *data, unsigned char *dataDepth)
{
    long index1, index2;
    for (int row = 0; row < cloud_in->height; row++)
    {
        for (int col = 0; col < cloud_in->width; col++)
        {
            index1 = 3 * (row * cloud_in->width + col);
            index2 = (cloud_in->height - row - 1) * cloud_in->width + col;

            //15.
            data[index1] = cloud_in->points[index2].r;
            data[index1 + 1] = cloud_in->points[index2].g;
            data[index1 + 2] = cloud_in->points[index2].b;

            //16.
            if (isnan(cloud_in->points[index2].x))
            {
                dataDepth[index1] = dataDepth[index1 + 1] = dataDepth[index1 + 2] = 0;
            }
            else
            {
                dataDepth[index1] = ((cloud_in->points[index2].x + 2) / 6.0) * 255.0;
                dataDepth[index1 + 1] = ((cloud_in->points[index2].y + 2) / 6.0) * 255.0;
                dataDepth[index1 + 2] = (cloud_in->points[index2].z / 10.0) * 255.0;
            }
        }
    }
}

void createVirtualCameras(osg::ref_ptr<osg::Camera> camera1, osg::ref_ptr<osg::Camera> camera2, osg::ref_ptr<osg::Geode> orthoTexture,
                          double camera_pitch, double camera_roll, double camera_height)
{

    //25.
    camera1->setCullingActive(false);
    camera1->setClearMask(0);
    camera1->setAllowEventFocus(false);
    camera1->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    camera1->setRenderOrder(osg::Camera::POST_RENDER, 0);
    camera1->setProjectionMatrix(osg::Matrix::ortho(0.0, 1.0, 0.0, 1.0, 0.5, 1000));
    camera1->addChild(orthoTexture.get());
    osg::StateSet *ss = camera1->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    //26.
    camera2->setCullingActive(false);
    camera2->setClearMask(0);
    camera2->setAllowEventFocus(false);
    camera2->setReferenceFrame(osg::Camera::ABSOLUTE_RF);
    camera2->setRenderOrder(osg::Camera::POST_RENDER, 1);
    camera2->setProjectionMatrixAsPerspective(50, 640. / 480., 0.5, 1000);

    //27.
    osg::Matrixd cameraRotation2;
    cameraRotation2.makeRotate(osg::DegreesToRadians(camera_pitch * 180 / M_PI), osg::Vec3(1, 0, 0),
                               osg::DegreesToRadians(0.0), osg::Vec3(0, 1, 0),
                               osg::DegreesToRadians(0.0), osg::Vec3(0, 0, 1));

    //28.
    osg::Matrixd cameraRotation3;
    cameraRotation3.makeRotate(osg::DegreesToRadians(0.0), osg::Vec3(1, 0, 0),
                               osg::DegreesToRadians(camera_roll * 180 / M_PI), osg::Vec3(0, 1, 0),
                               osg::DegreesToRadians(0.0), osg::Vec3(0, 0, 1));

    //29.
    osg::Matrixd cameraTrans;
    cameraTrans.makeTranslate(0, 0, camera_height * 100.0);

    //30.
    osg::Matrix matrix_view;
    matrix_view.makeLookAt(osg::Vec3(0, 0, 0), osg::Vec3(0, -1, 0), osg::Vec3(0, 0, 1));
    osg::Matrixd cameraRotation;
    cameraRotation.makeRotate(osg::DegreesToRadians(180.0), osg::Vec3(0, 0, 1),
                              osg::DegreesToRadians(-90.0), osg::Vec3(1, 0, 0),
                              osg::DegreesToRadians(0.0), osg::Vec3(0, 1, 0));

    //31.
    camera2->setViewMatrix(osg::Matrix::inverse(cameraRotation * cameraRotation3 * cameraRotation2 * cameraTrans));

    //32.
    camera1->setViewport(0, 0, 640, 480);
    camera2->setViewport(0, 0, 640, 480);
}

void createOrthoTexture(osg::ref_ptr<osg::Geode> orthoTextureGeode,
                        unsigned char *data, unsigned char *dataDepth, int width, int height)
{
    //18.
    osg::Image *image = new osg::Image;
    osg::Image *imageDepth = new osg::Image;
    image->setOrigin(osg::Image::TOP_LEFT);
    imageDepth->setOrigin(osg::Image::TOP_LEFT);
    image->setImage(width, height, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, data, osg::Image::NO_DELETE);
    imageDepth->setImage(width, height, 1, GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, dataDepth, osg::Image::NO_DELETE);

    //19.
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    osg::ref_ptr<osg::Texture2D> texture2 = new osg::Texture2D;
    texture->setImage(image);
    texture2->setImage(imageDepth);

    //20.
    osg::ref_ptr<osg::Drawable> quad = osg::createTexturedQuadGeometry(osg::Vec3(), osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
    quad->getOrCreateStateSet()->setTextureAttributeAndModes(0, texture.get());
    quad->getOrCreateStateSet()->setTextureAttributeAndModes(1, texture2.get());

    //21.
    osg::ref_ptr<osg::Shader> vertShader = new osg::Shader(osg::Shader::VERTEX, textureVertexSource);
    osg::ref_ptr<osg::Shader> fragShader = new osg::Shader(osg::Shader::FRAGMENT, textureFramentSource);
    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader(fragShader.get());
    program->addShader(vertShader.get());

    //22.
    quad->setDataVariance(osg::Object::DYNAMIC);
    quad->getOrCreateStateSet()->setAttributeAndModes(program.get());
    quad->getOrCreateStateSet()->addUniform(new osg::Uniform("texture", 0));
    quad->getOrCreateStateSet()->addUniform(new osg::Uniform("textureDepth", 1));
    orthoTextureGeode->addDrawable(quad.get());
}

void detectCollisions(osg::ref_ptr<osg::PositionAttitudeTransform> helicopterTransf,
                      pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree,
                      bool *collidedLeft, bool *collidedRight, bool *collidedFront, bool *collidedBack, bool *collidedBelow)
{

    std::vector<int> search_indexes;
    std::vector<float> search_radiuses;

    //45.
    pcl::PointXYZRGBA helicopter;
    helicopter.x = -helicopterTransf->getPosition().x() / 100.f;
    helicopter.y = -helicopterTransf->getPosition().z() / 100.f;
    helicopter.z = -helicopterTransf->getPosition().y() / 100.f;

    //47.
    pcl::PointXYZRGBA helicopterLeftNeighborPoint;
    helicopterLeftNeighborPoint.x = helicopter.x - 0.05;
    helicopterLeftNeighborPoint.y = helicopter.y - 0.05;
    helicopterLeftNeighborPoint.z = helicopter.z;

    pcl::PointXYZRGBA helicopterRightNeighborPoint;
    helicopterRightNeighborPoint.x = helicopter.x + 0.05;
    helicopterRightNeighborPoint.y = helicopter.y - 0.05;
    helicopterRightNeighborPoint.z = helicopter.z;

    pcl::PointXYZRGBA helicopterFrontNeighborPoint;
    helicopterFrontNeighborPoint.x = helicopter.x;
    helicopterFrontNeighborPoint.y = helicopter.y - 0.05;
    helicopterFrontNeighborPoint.z = helicopter.z + 0.05;

    pcl::PointXYZRGBA helicopterBackNeighborPoint;
    helicopterBackNeighborPoint.x = helicopter.x;
    helicopterBackNeighborPoint.y = helicopter.y - 0.05;
    helicopterBackNeighborPoint.z = helicopter.z - 0.05;

    pcl::PointXYZRGBA helicopterBelowNeighborPoint;
    helicopterBelowNeighborPoint.x = helicopter.x;
    helicopterBelowNeighborPoint.y = helicopter.y;
    helicopterBelowNeighborPoint.z = helicopter.z - 0.1;

    //48.
    kdtree->radiusSearch(helicopterLeftNeighborPoint, 0.05, search_indexes, search_radiuses);

    //49.
    *collidedLeft = search_indexes.size() > 0;
    kdtree->radiusSearch(helicopterRightNeighborPoint, 0.05, search_indexes, search_radiuses);
    *collidedRight = search_indexes.size() > 0;
    kdtree->radiusSearch(helicopterFrontNeighborPoint, 0.05, search_indexes, search_radiuses);
    *collidedFront = search_indexes.size() > 0;
    kdtree->radiusSearch(helicopterBackNeighborPoint, 0.05, search_indexes, search_radiuses);
    *collidedBack = search_indexes.size() > 0;
    kdtree->radiusSearch(helicopterBelowNeighborPoint, 0.1, search_indexes, search_radiuses);
    *collidedBelow = search_indexes.size() > 0;
}

int main(int argsc, char **argsv)
{

    // helicopter dynamics state variables
    bool collidedLeft = false, collidedRight = false, collidedFront = false, collidedBack = false, collidedBelow = false;

    // reading point cloud
    //1.

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (pcl::io::loadPCDFile(argsv[1], *cloud_in) == -1)
    {
        PCL_ERROR("Couldn't read the PCD file \n");
        return (-1);
    }

    // estimate the camera pose w.r.t. to ground plane
    //2.
    double camera_pitch, camera_roll, camera_height;

    estimateCameraPose(cloud_in, &camera_pitch, &camera_roll, &camera_height);
    // rotate point cloud so as to align the ground plane with a virtual's ground plane
    //10.

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZRGBA>);
    rotatePointCloud(cloud_in, cloud_rotated, camera_pitch, camera_roll, camera_height);

    // go through the point cloud and generate a RGB image and a range image
    //14.
    int size = cloud_in->height * cloud_in->width * 3;
    unsigned char *data = (unsigned char *)calloc(size, sizeof(unsigned char));
    unsigned char *dataDepth = (unsigned char *)calloc(size, sizeof(unsigned char));
    createImageFromPointCloud(cloud_in, data, dataDepth);

    // create a texture from the RGB image and use depth data to fill the z-buffer
    //17.
    osg::ref_ptr<osg::Geode> orthoTextureGeode = new osg::Geode;
    createOrthoTexture(orthoTextureGeode, data, dataDepth, cloud_in->width, cloud_in->height);

    // create an orthographic camera imaging the texture and a perspective camera based on the camera's pose and intrinsics
    //24.
    osg::ref_ptr<osg::Camera> camera1 = new osg::Camera;
    osg::ref_ptr<osg::Camera> camera2 = new osg::Camera;
    createVirtualCameras(camera1, camera2, orthoTextureGeode, camera_pitch, camera_roll, camera_height);

    // add the two cameras to the scene's root node
    //33.
    osg::ref_ptr<osg::Group> root = new osg::Group;
    root->addChild(camera1.get());
    root->addChild(camera2.get());

    // create a dynamic helicopter node alongside its line
    //36.
    osg::ref_ptr<osg::PositionAttitudeTransform> helicopterTransf = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::PositionAttitudeTransform> lineTransf = new osg::PositionAttitudeTransform;
    osg::ref_ptr<osg::PositionAttitudeTransform> circleTransf = new osg::PositionAttitudeTransform;
    CreateHelicopter(helicopterTransf, lineTransf, circleTransf);

    // run a controller to allow the user to control the helicopter with the keyboard
    //37.
    osg::ref_ptr<HelicopterController> ctrler = new HelicopterController(helicopterTransf.get(), &collidedLeft,
                                                                         &collidedRight, &collidedFront, &collidedBack, &collidedBelow);

    // force the perspective camera look at the helicopter and the line
    //38.
    camera2->addChild(helicopterTransf);
    camera2->addChild(lineTransf);
    camera2->addChild(circleTransf);

    // create a root's viewer
    //34.
    osgViewer::Viewer viewer;
    viewer.setUpViewInWindow(0, 0, 640, 480);
    viewer.setSceneData(root.get());

    //38.A.
    viewer.addEventHandler(ctrler.get());

    // create a kdtree from the point cloud in order to speed up collision detection
    //44.
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
    kdtree->setInputCloud(cloud_rotated);

    //35.
    while (!viewer.done())
    {
        //43.
        detectCollisions(helicopterTransf, kdtree, &collidedLeft, &collidedRight, &collidedFront, &collidedBack, &collidedBelow);
        viewer.frame();
    }
}
