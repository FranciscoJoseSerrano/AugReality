#include "models/cloudInitializer.h"

int main(int argsc, char **argsv)
{   
    CloudInitializer *initializer = new CloudInitializer();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = initializer->initializePointCloud("../clouds/treino/objecto_caixa_azul.pcd", "objecto_caixa_azul.pcd");
    if (cloud == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }

    initializer->FindObjectAndCreateFile(cloud, "caixa_azul");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 = initializer->initializePointCloud("../clouds/treino/objecto_caixa_branca.pcd", "objecto_caixa_branca.pcd");
    if (cloud2 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud2, "caixa_branca");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3 = initializer->initializePointCloud("../clouds/treino/objecto_caixa_vermelha.pcd", "objecto_caixa_vermelha.pcd");
    if (cloud3 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud3, "caixa_vermelha");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4 = initializer->initializePointCloud("../clouds/treino/objecto_cassete_preta.pcd", "objecto_cassete_preta.pcd");
    if (cloud4 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud4, "cassete_preta");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud5 = initializer->initializePointCloud("../clouds/treino/objecto_furador.pcd", "objecto_furador.pcd");
    if (cloud5 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud5, "furador");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud6 = initializer->initializePointCloud("../clouds/treino/objecto_livro_azul.pcd", "objecto_livro_azul.pcd");
    if (cloud6 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud6, "livro_azul");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud7 = initializer->initializePointCloud("../clouds/treino/objecto_livro_cinzento.pcd", "objecto_livro_cinzento.pcd");
    if (cloud7 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud7, "livro_cinzento");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud8 = initializer->initializePointCloud("../clouds/treino/objecto_xicara_branca.pcd", "objecto_xicara_branca.pcd");
    if (cloud8 == NULL)
    {
        PCL_ERROR("Cloudn't read file. \n");
        return (-1);
    }
    initializer->FindObjectAndCreateFile(cloud8, "xicara_branca");
}