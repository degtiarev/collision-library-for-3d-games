#ifndef COLLISION_LIBRARY_H
#define COLLISION_LIBRARY_H

// collision library interface
#include <collision_interface.h>

#include <typeinfo>
#include <unordered_map>

#include <iostream>
#include <fstream>


namespace collision
{

// "Declaration" of the struct
struct StateChangeObj;


class MyController : public Controller {
    GM_SCENEOBJECT (MyController)
    public:

        explicit MyController () = default;

    void add (DynamicPSphere* const sphere);
    void add (StaticPSphere* const sphere);
    void add (StaticPPlane* const plane);
    void add (StaticPCylinder* const cylinder);
    void add (StaticPBezierSurf* const surf);

    std::unordered_set<StaticPPlane *> getAttachedObjects(DynamicPSphere* sphere);

    // States
    void detectStateChanges(double dt);
    StateChangeObj detectStateChange(DynamicPSphere* sphere, double dt);
    void handleStates (StateChangeObj& state, double dt);
    GMlib::Vector<float,3> getClosestPoint(DynamicPSphere* S, seconds_type dt);
    void handleCollision ( collision::CollisionObject& col, double dt);

    Environment                                 _stillEnvironment;
    DefaultEnvironment                          _env;


private:

    template <typename T_s>
    void dynamicCollision(T_s* sphere, seconds_type dt);

    template <class Container>
    void sortAndMakeUniqueStates(Container& c);

    template <typename Container_1, typename Container_2>
    void crossUnique( Container_1 container1, Container_2 container2);

protected:
    void localSimulate (double dt) override;

    std::vector<DynamicPSphere*>                _dynamic_spheres;
    std::vector<StaticPSphere*>                 _static_spheres;
    std::vector<StaticPPlane*>                  _static_planes;
    std::vector<StaticPCylinder*>               _static_cylinders;
    std::vector<StaticPBezierSurf*>             _static_bezier_surf;

    std::vector<collision::CollisionObject>     _collisions;
    std::vector<StateChangeObj>                 _singularities;

    std::unordered_map<DynamicPSphere*, std::unordered_set<StaticPPlane*>>     _map;

};

template <>
class DynamicPhysObject<GMlib::PSphere<float>> : public DynamicPhysObject_Base<GMlib::PSphere<float>> {
public:
    using DynamicPhysObject_Base<GMlib::PSphere<float>>::DynamicPhysObject_Base;

    enum class States {
        Free,
        Rolling,
        AtRest,
        NoChange
    };

    MyController*   _sphereController;
    States          _state = States::Free;     // Which state is the sphere in


    GMlib::Vector<double,3>
    adjustedTrajectory (seconds_type dt);

    void
    simulateToTInDt( seconds_type t ) override;

    GMlib::Vector<double, 3>
    computeTrajectory (seconds_type dt) const override; // [m]

    GMlib::Vector<double, 3>
    externalForces () const override; // [m / s^2]

};

template <>
class StaticPhysObject<GMlib::PPlane<float>> :  public PhysObject<GMlib::PPlane<float>, PhysObjectType::Static> {
public:
    int id=0;

    void moveUp();
    void moveDown();
    void moveLeft();
    void moveRight();
    using PhysObject<GMlib::PPlane<float>, PhysObjectType::Static>::PhysObject;


    void simulateToTInDt (seconds_type) override {}

    int getId() const;
    void setId(int value);

};

// StateChangeObject struct
struct StateChangeObj {
    DynamicPSphere*                             obj;               // Object whos state will change
    std::unordered_set<StaticPPlane*>           attachedPlanes;     // Object that obj1 will change state ACCORDING to
    seconds_type                                time;               // Time of singularity
    DynamicPSphere::States                      state;              // State that obj1 will change to

    StateChangeObj
    (DynamicPSphere* o1, std::unordered_set<StaticPPlane*> planes, seconds_type t, DynamicPSphere::States s) :
        obj{o1}, attachedPlanes{planes}, time{t} ,state{s} {}
};

template <class PSurf_T, typename... Arguments>
std::unique_ptr<DynamicPhysObject<PSurf_T>> unittestDynamicPhysObjectFactory(Arguments... parameters) {

    return std::make_unique<DynamicPhysObject<PSurf_T>> (parameters...);
}

template <class PSurf_T, typename... Arguments>
std::unique_ptr<StaticPhysObject<PSurf_T>> unittestStaticPhysObjectFactory(Arguments... parameters) {

    return std::make_unique<StaticPhysObject<PSurf_T>> (parameters...);
}

template <typename Container_1, typename Container_2>
void MyController::crossUnique(Container_1 ColContainer, Container_2 stateContainer) {

    std::vector<collision::CollisionObject>         _newCollisions;
    std::vector<collision::StateChangeObj>          _newStateOjects;

    auto amIinCollision = [](Container_1 a, const auto& b){
        for(auto& c : a) {
            if(c.obj1==b.obj1) return true;
            if(c.obj1==b.obj2) return true;
            if(c.obj2==b.obj1) return true;
            if(c.obj2==b.obj2) return true;
            return false;
        }

    };

    auto amIinState = [](Container_2 a, const auto& b){
        for(auto& d:a){
            if(d.obj==b.obj) return true;
            return false;
        }
    };

    auto objPred = [](const auto& a, const auto& b) {
        if(a.obj1 == b.obj or a.obj2 == b.obj ) return true;
        return false;
    };

    auto timePred = [](const auto&a, const auto&b) {
        if(a.t_in_dt < b.time ) return true;
        return false;
    };

    bool colBigger;

    if( ColContainer.size() > stateContainer.size() ) {

        colBigger = true;

    }
    else colBigger = false;

    if( colBigger == true ) {

        bool placed = false;

        for(auto firstIter = std::end(ColContainer) - 1; firstIter!= std::begin(ColContainer) - 1;--firstIter)
        {
            for(auto secondIter = std::end(stateContainer) - 1; secondIter!= std::begin(stateContainer) - 1;--secondIter)
            {
                placed = false;

                //Check for same Objects
                if(objPred(*firstIter,*secondIter))
                {
                    //check for time of objects in both container
                    if(timePred(*firstIter,*secondIter))
                    {
                        //check if already in new container
                        if(amIinCollision(_newCollisions,*firstIter)==false) {
                            _newCollisions.push_back(*firstIter);
                            placed = true;
                        }
                    }

                    else
                    {
                        //check if already in new container
                        if(amIinState(_newStateOjects,*secondIter)==false)
                        {
                            _newStateOjects.push_back(*secondIter);
                            placed = true;
                        }
                    }
                }
            }

            // If col object NOT in states
            if (placed == false) _newCollisions.push_back(*firstIter);

        }
 // Does the same thing for the Smaller container as the if(placed == false) check above does for the Larger container
        for( auto& state : stateContainer) {

            if(amIinState(_newStateOjects, state) == false ) _newStateOjects.push_back(state);
        }
    }

    // States are bigger
    else {

        bool placed = false;

        for(auto firstIter = std::end(stateContainer) - 1; firstIter!= std::begin(stateContainer) - 1;--firstIter)
        {
            for(auto secondIter = std::end(ColContainer) - 1; secondIter!= std::begin(ColContainer) - 1;--secondIter)
            {

                placed = false;

                //Check for same Objects
                if(objPred(*secondIter,*firstIter))
                {
                    //check for time of objects in both container
                    if(timePred(*secondIter,*firstIter))
                    {
                        //check if already in new container
                        if(amIinCollision(_newCollisions,*secondIter)==false) {
                            _newCollisions.push_back(*secondIter);
                            placed = true;
                        }
                    }

                    else
                    {
                        //check if already in new container
                        if(amIinState(_newStateOjects,*firstIter)==false)
                        {
                            _newStateOjects.push_back(*firstIter);
                            placed = true;
                        }
                    }
                }
            }

            // If col object NOT in states
            _newStateOjects.push_back(*firstIter);
        }

        for( auto& collision : ColContainer) {

            if(amIinCollision(_newCollisions, collision) == false ) _newCollisions.push_back(collision);
        }

    }
}

// Sort and make Unique for states
template <class Container>
void MyController::sortAndMakeUniqueStates(Container& c) {

    // Sorting
    std::sort( std::begin(c), std::end(c), [] (const auto& a, const auto&b) {
        return a.time < b.time;
    });

    // Make unique

    auto pred =  [](const auto &a, const auto &b) {

        if( a.obj == b.obj ) return true;

        return false;
    };

    typename Container::iterator NE = std::end(c);
    for( auto first_iter = std::begin(c); first_iter != NE; ++first_iter) {

        for( auto r_iterator = NE - 1; r_iterator != first_iter; --r_iterator) {

            if( (pred(*first_iter, *r_iterator))) {
                std::swap( *r_iterator, *(NE-1) );
                NE--;

            }
        }
    }

    c.erase( NE, std::end( c ) );
}

template <class Container_T >
void sortAndMakeUnique( Container_T& container) {

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

        if( a.obj1 == b.obj1 ) return true;
        if( a.obj1 == b.obj2 ) return true;
        if( a.obj2 == b.obj1 ) return true;
        if( ( is_d_pred(a.obj2) or is_d_pred(b.obj2) )
                and a.obj2 == b.obj2 ) return true;

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

} // EOF


template <typename T_s>
inline void MyController::dynamicCollision(T_s* dyn_obj, seconds_type dt) {

    bool movingObject = false;

    for( auto& s : _dynamic_spheres ) {
        if( s->_state != DynamicPSphere::States::AtRest ) {
            movingObject = true;
            break;
        }
    }

    if( movingObject ) {

        // Dynamic Object vs. Sphere
        for( auto iter = std::begin(_dynamic_spheres); iter != std::end(_dynamic_spheres); ++iter) {

            // Check only for collisions for the first dynamic sphere
            auto col = detectCollision(*dyn_obj, **iter, dt);

            // The check for if the second object is a dynamic sphere is done in the main algorithm
            // and the method is called again, only with the 1'st and 2'nd sphere swapped

            if( col.CollisionState::flag == CollisionStateFlag::Collision ) {

                auto& first_sphere = dyn_obj;
                auto& second_sphere = *iter;

                auto new_t = std::max(first_sphere->curr_t_in_dt, second_sphere->curr_t_in_dt);

                if( col.time > seconds_type(new_t) and col.time < dt) {
                    auto col_obj = CollisionObject(first_sphere, second_sphere, col.time);
                    _collisions.push_back(col_obj);


                }
            }
        }

        // Dynamic Object vs. Planes
        for( auto iter = std::begin(_static_planes); iter != std::end(_static_planes); ++iter) {

            if( dyn_obj->_state != DynamicPSphere::States::AtRest) {

                auto col = detectCollision(*dyn_obj, **iter, dt);

                if( col.CollisionState::flag == CollisionStateFlag::Collision ) {

                    auto new_t = dyn_obj->curr_t_in_dt;

                    if( col.time > seconds_type(new_t) and col.time < dt)
                    {
                        auto col_obj = CollisionObject(dyn_obj, *iter, col.time);
                        _collisions.push_back(col_obj);
                    }
                }
            }
        }
    }
}

} // END namespace collision



#endif




