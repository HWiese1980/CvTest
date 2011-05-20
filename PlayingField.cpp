/* 
 * File:   PlayingField.cpp
 * Author: ros
 * 
 * Created on 20. Mai 2011, 16:25
 */

#include "PlayingField.h"
#include <iostream>

PlayingField::PlayingField(int sizeX, int sizeY)
{
    std::cout << "INITIALIZING PLAYING FIELD" << std::endl;
    
    this->create(sizeY, sizeY, CV_8U);
    this->Clear();
    
    mRoot = new Ogre::Root("", "", "cvtest_ogre.log");
    
    Ogre::StringVector req_plugins;
    req_plugins.push_back("GL RenderSystem");
    req_plugins.push_back("Octree & Terrain Scene Manager");
    
    Ogre::StringVector load_plugins;
    load_plugins.push_back("/usr/lib/OGRE/RenderSystem_GL");
    load_plugins.push_back("/usr/lib/OGRE/Plugin_OctreeSceneManager");
    
    for(Ogre::StringVector::iterator j = load_plugins.begin(); j != load_plugins.end(); j++)
    {
        mRoot->loadPlugin(*j);
    }
    
    Ogre::RenderSystemList* lst = mRoot->getAvailableRenderers();
    Ogre::RenderSystem* sys = NULL;

    for(Ogre::RenderSystemList::iterator j = lst->begin(); j != lst->end(); j++)
    {
        std::cout << "Available Rendering System: " << (*j)->getName() << std::endl;
    }

    if(lst->empty())
    {
        std::cerr << "ERROR: NO RENDERING SYSTEMS FOUND" << std::endl;
    }
    else
    {
        sys = (*lst->begin());
        std::cout << "Chosen Renderer: " << sys->getName() << std::endl;
    }
    
    if(sys != NULL) 
    {
        mRoot->setRenderSystem(sys);
        mRoot->initialise(false, "no", "");
        mRoot->createRenderWindow("3D", 640, 480, false, NULL);
    }
    
    std::cout << "PLAYING FIELD INITIALIZED" << std::endl;
}

PlayingField::PlayingField(const PlayingField& orig)
{
}

PlayingField::~PlayingField()
{
}

void PlayingField::Clear()
{
    this->setTo(cv::Scalar(0));
}

void PlayingField::Render()
{
    
}

const bool PlayingField::operator ()(int x, int y)
{
    return (this->at<uchar>(y, x) != 0);
}

void PlayingField::SetAt(int x, int y, bool value)
{
    if(x >= 0 && x < this->cols && y >= 0 && y < this->rows) *(this->ptr(y, x)) = (value ? 1 : 0);
}

std::ostream& operator<<(std::ostream& s, PlayingField& field)
{
    for(int row = 0; row < field.rows; row++)
    {
        for(int col = 0; col < field.cols; col++)
        {
            s << "[" << (field(col, row) == 1 ? 'X' : ' ')  << "]";
        }
        s << std::endl;
    }
    return s;
}



