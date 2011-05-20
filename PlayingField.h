/* 
 * File:   PlayingField.h
 * Author: ros
 *
 * Created on 20. Mai 2011, 16:25
 */

#ifndef PLAYINGFIELD_H
#define	PLAYINGFIELD_H

#include <opencv/cv.h>
#include <OGRE/Ogre.h>

class PlayingField : public cv::Mat {
public:
    PlayingField(int sizeX, int sizeY);
    PlayingField(const PlayingField& orig);
    virtual ~PlayingField();
    
    void Clear();
    void SetAt(int x, int y, bool value);
    void Render();
    
    const bool operator()(int x, int y);
    
private:
    Ogre::Root* mRoot;
    
};

std::ostream& operator<<(std::ostream& s, PlayingField& field);



#endif	/* PLAYINGFIELD_H */

