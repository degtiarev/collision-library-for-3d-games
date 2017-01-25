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
    GMlib::Vector<double, 3> computeTrajectory (seconds_type dt) const override;
    GMlib::Vector<double, 3> externalForces () const  override;
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

template <class Container_T > void sortAndMakeUnique( Container_T& container) {

    // Sort

    std::sort( std::begin(container), std::end(container), [](const auto& a, const auto& b) {
        return a.t_in_dt < b.t_in_dt;
    });

    // Make unique

    auto pred =  [](const auto &a, const auto &b) {

        auto is_d_pred = []( const auto* obj ) {
            if(dynamic_cast<const DynamicPSphere*>(obj)) return true;

            return false;
        };

        if(( a.obj1 == b.obj1 ) or ( a.obj1 == b.obj2 ) or ( a.obj2 == b.obj1 ) or
                ( ( is_d_pred(a.obj2) or is_d_pred(b.obj2) )) and a.obj2 == b.obj2 ) return true;

        return false;
    };

    typename Container_T::iterator NE = std::end(container);
    for( auto first_iter = std::begin(container); first_iter != NE; ++first_iter) {

        for( auto r_iterator = NE - 1; r_iterator != first_iter; --r_iterator) {

            if( (pred(*first_iter, *r_iterator))) {
                std::swap( *r_iterator, *(NE-1) );
                NE--;

            }
        }
    }

    container.erase( NE, std::end( container ) );
}

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
