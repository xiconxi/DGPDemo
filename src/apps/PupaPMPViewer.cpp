//
// Created by pupa on 11/30/20.
//
#include <entityx/entityx.h>
#include "GLComponent.h"

#include <pmp/visualization/TrackballViewer.h>
#include <pmp/Types.h>
#include <iostream>
namespace ex = entityx;






class Viewer: public pmp::TrackballViewer, public ex::EntityX {
public:
    explicit Viewer(): pmp::TrackballViewer("Pupa Viewer", 800, 600, true){
        systems.configure();
    }

    void draw(const std::string& drawMode) override{
        systems.update_all(0.1);
    }


private:

};

int main()  {
    Viewer viewer;
    viewer.run();
}


