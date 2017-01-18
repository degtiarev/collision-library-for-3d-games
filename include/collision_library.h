#ifndef COLLISION_LIBRARY_H
#define COLLISION_LIBRARY_H

// collision library interface
#include <collision_interface.h>



namespace collision
{


template <>
class DynamicPhysObject<GMlib::PSphere<float>> : public DynamicPhysObject_Base<GMlib::PSphere<float>> {
public:
    using DynamicPhysObject_Base<GMlib::PSphere<float>>::DynamicPhysObject_Base;
    void    simulateToTInDt( seconds_type t ) override;
     GMlib::Vector<float, 3> computeTrajectory (seconds_type dt) const override;
      GMlib::Vector<float, 3> externalForces () const  override;
};


class MyController : public Controller {
    GM_SCENEOBJECT(MyController)
    public:
        void add (DynamicPSphere* const sphere) { _dynamic_spheres.push_back(sphere); }
        void add (StaticPSphere* const sphere) { _static_spheres.push_back(sphere); }
        void add (StaticPPlane* const plane) { _static_planes.push_back(plane); }
        void add (StaticPCylinder* const cylinder) { _static_cylinders.push_back(cylinder); }
        void add (StaticPBezierSurf* const surf) { _static_bezier_surf.push_back(surf); }


    protected:
        std::vector<DynamicPSphere*>    _dynamic_spheres;
        std::vector<StaticPSphere*>     _static_spheres;
        std::vector<StaticPPlane*>      _static_planes;
        std::vector<StaticPCylinder*>   _static_cylinders;
        std::vector<StaticPBezierSurf*> _static_bezier_surf;

        std::vector<collision::CollisionObject> _collisions;
    };

template <class Container_T > void sortAndMakeUnique( Container_T& container){}

template <class PSurf_T, typename... Arguments>
std::unique_ptr<DynamicPhysObject<PSurf_T>> unittestDynamicPhysObjectFactory(Arguments... parameters){

    return std::make_unique<DynamicPhysObject<PSurf_T>>(parameters...);
}

template <class PSurf_T, typename... Arguments>
std::unique_ptr<StaticPhysObject<PSurf_T>> unittestStaticPhysObjectFactory(Arguments... parameters){


     return std::make_unique<StaticPhysObject<PSurf_T>>(parameters...);

}


} // END namespace collision



#endif //COLLISION_LIBRARY_H
