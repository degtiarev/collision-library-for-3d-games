#include "collision_library.h"

#include <chrono>
using namespace std::chrono_literals;


namespace collision
{

#define Collison_detect_respons_method {
// dynamic sphere - dynamic sphere
CollisionState detectCollision (DynamicPhysObject<GMlib::PSphere<float>>&      S0,
                                DynamicPhysObject<GMlib::PSphere<float>>&      S1,
                                seconds_type                                   dt)
{
    const auto dt_max = dt;
    const auto dt_min = std::max(S0.curr_t_in_dt, S1.curr_t_in_dt);
    const auto new_dt = dt_max - dt_min;

    const auto S0_position = S0.getMatrixToScene() * S0.getPos();
    const auto S1_position = S1.getMatrixToScene() * S1.getPos();

    const auto S0_radius = S0.getRadius();
    const auto S1_radius = S1.getRadius();
    const auto radius_sum = S0_radius + S1_radius;

    auto r1 = S1.computeTrajectory(new_dt);
    auto r2 = S0.computeTrajectory(new_dt);

    // If the sphere1 state is Rolling, it should use an adjusted DS
    if( S1._state == DynamicPSphere::States::Rolling )
        r1 = S1.adjustedTrajectory(new_dt);

    // If the sphere2 state is Rolling, it should use an adjusted DS
    if( S0._state == DynamicPSphere::States::Rolling )
        r2 = S0.adjustedTrajectory(new_dt);

    const auto Q = (S1_position - S0_position);

    const auto R = r1 - r2;

    const auto _QR = Q * R;
    const auto _QRQR = std::pow( _QR, 2);

    const auto _RR = R * R;
    const auto _QQ = Q * Q;

    const auto _rr = std::pow( radius_sum, 2);
    const auto _square = std::sqrt(_QRQR - (_RR * (_QQ - _rr)));

    const auto epsilon = 1e-7;

    if ( _square < 0 )
        return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityNoCollision);

    else if ( (_QQ - _rr) < epsilon )
        return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityParallelAndTouching);

    else if ( _RR < epsilon )
        return CollisionState( seconds_type(0.0), CollisionStateFlag::SingularityParallel);

    const auto x = (-_QR - _square) / _RR;
    return CollisionState(((x * new_dt) + dt_min), CollisionStateFlag::Collision);
}

// dynamic sphere - dynamic sphere
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>&          S0,
                            DynamicPhysObject<GMlib::PSphere<float>>&          S1,
                            seconds_type                                       dt)
{
    const auto S0_old_vel = S0.velocity;    // 2.1
    const auto S1_old_vel = S1.velocity;    // -2.1

    const auto S0_pos = S0.getPos().toType<double>();
    const auto S1_pos = S1.getPos().toType<double>();

    const auto S0_mass = S0.mass;
    const auto S1_mass = S1.mass;

    const auto distance_vector_d = GMlib::Vector<double,3>(S1_pos - S0_pos);
    const auto normal_d = distance_vector_d.getNormalized();
    const auto n = (GMlib::Vector<double,3>(distance_vector_d).getLinIndVec()).getNormalized();

    const auto v0_d = (S0_old_vel * normal_d);
    const auto v1_d = (S1_old_vel * normal_d);
    const auto v0_n = (S0_old_vel * n);
    const auto v1_n = (S1_old_vel * n);

    const auto new_v0_d = (((S0_mass - S1_mass) / (S0_mass + S1_mass) ) * v0_d ) + (((2 * S1_mass) / (S0_mass + S1_mass) ) * v1_d );
    const auto new_v1_d = (((S1_mass - S0_mass) / (S0_mass + S1_mass) ) * v1_d ) + (((2 * S0_mass) / (S0_mass + S1_mass) ) * v0_d );

    const auto S0_new_vel = (v0_n * n) + (new_v0_d * normal_d);  // -2.1
    const auto S1_new_vel = v1_n * n + new_v1_d * normal_d;     // 2.1

    S0.velocity = S0_new_vel;
    S1.velocity = S1_new_vel;
}

// dynamic sphere - static plane
CollisionState detectCollision (DynamicPhysObject<GMlib::PSphere<float>>&       S,
                                const StaticPhysObject<GMlib::PPlane<float>>&   P,
                                seconds_type                                    dt)
{
    const auto dt_max = dt;
    const auto dt_min = S.curr_t_in_dt;
    const auto new_dt = dt_max - dt_min;

    const auto s_position = S.getMatrixToScene() * S.getPos();
    const auto s_radius = S.getRadius();

    auto &plane = const_cast<StaticPhysObject<GMlib::PPlane<float>>&>(P);
    const auto p = plane.evaluateParent(0.5f, 0.5f, 1, 1);                      // plane.getMatrixToScene * plane.evaluateParent(0.5f, 0.5f, 1, 1)
    const auto plane_pos = p(0)(0);
    const auto u = p(1)(0);
    const auto v = p(0)(1);

    const auto n = u ^ v;
    const auto n_normal = GMlib::Vector<float,3>(n).getNormalized();

    const auto d = (plane_pos + s_radius * n_normal) - s_position;

    auto ds = S.computeTrajectory(new_dt);

    // If the sphere's state is Rolling, it should use an adjusted DS
    if( S._state == DynamicPSphere::States::Rolling )
        ds = S.adjustedTrajectory(new_dt);

    const auto Q = (d * n_normal);
    const auto R = ( ds * n_normal );

    const auto epsilon = 1e-7;

    // Check The sphere is "touching" the surface
    if( std::abs(Q) < epsilon )
        return CollisionState( seconds_type(0.0), CollisionStateFlag::SingularityParallelAndTouching);

    else if( std::abs(R) < epsilon)
        return CollisionState( seconds_type(0.0), CollisionStateFlag::SingularityParallel);

    const auto x = Q / R;

    return CollisionState( (x * new_dt) + dt_min);
}

// dynamic sphere - static plane
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>&           S,
                            const StaticPhysObject<GMlib::PPlane<float>>&       P,
                            seconds_type                                        dt) {

    auto &plane = const_cast<StaticPhysObject<GMlib::PPlane<float>>&>(P);
    const auto p = plane.evaluateParent(0.5f, 0.5f, 1, 1);
    const auto u = p(1)(0);
    const auto v = p(0)(1);

    const auto n = u ^ v;
    const auto n_normal = GMlib::Vector<float,3>(n).getNormalized();

    auto new_vel = S.velocity -  (2*(S.velocity * n_normal) * n_normal)*1; //* 0.95; - reducing force

    S.velocity = new_vel;
}

// dynamic sphere - static Bezier surface
CollisionState detectCollision (const DynamicPSphere&                           S,
                                const StaticPBezierSurf&                        B,
                                seconds_type                                    dt) {

    const auto dt_max = dt;
    const auto dt_min = S.curr_t_in_dt;
    const auto new_dt = dt_max - dt_min;

    const auto p0 = S.getPos();
    const auto r = S.getRadius();

    float u, v, t;
    u = 0.5;
    v = 0.5;
    t = 0.0;
    const auto epsilon = 1e-5;

    for ( int i = 0; i < 50; i++) {

        auto ds = S.computeTrajectory( new_dt );
        auto &Surf = const_cast<StaticPBezierSurf&>(B);
        const auto surf = Surf.evaluate(u, v, 1, 1);
        const auto p = p0 + ds*t;
        const auto q = surf(0)(0);
        const auto Su = surf(1)(0);
        const auto Sv = surf(0)(1);
        const auto Sn = GMlib::Vector<float,3>(Su ^ Sv).getNormalized();


        GMlib::SqMatrix<float,3> A;
        A.setCol(Su, 0);
        A.setCol(Sv, 1);
        A.setCol(-ds, 2);
        auto A_inv = A;

        A_inv.invert();

        const auto b = GMlib::Vector<float,3> {p-q-Sn*r};

        const auto x = A_inv * b;

        const auto deltaU = x(0);
        const auto deltaV = x(1);
        const auto deltaT = x(2);

        u += deltaU;
        v += deltaV;
        t += deltaT;

        if( (std::abs(deltaU) < epsilon) and (std::abs(deltaV) < epsilon) and (std::abs(deltaT) < epsilon) )

            return CollisionState(seconds_type(deltaT), CollisionStateFlag::Collision);

    }

    return CollisionState(seconds_type(dt_min), CollisionStateFlag::SingularityNoCollision);}
//**********************************************************************************************
#define end1  }

#define Adding_objects_to_vector {

void
MyController::add(DynamicPSphere * const sphere) {

    sphere->environment = &_env;
    _dynamic_spheres.push_back(sphere);
    _map[sphere];

}
void
MyController::add(StaticPSphere * const sphere) {

    _static_spheres.push_back(sphere);
}
void
MyController::add(StaticPPlane * const plane) {

    _static_planes.push_back(plane);
}
void
MyController::add(StaticPCylinder * const cylinder) {

    _static_cylinders.push_back(cylinder);
}
void
MyController::add(StaticPBezierSurf * const surf) {

    _static_bezier_surf.push_back(surf);
}
//**********************************************************************************************
#define end1 }

#define id_setter_and_getter {
// id setter and getter
int StaticPhysObject<GMlib::PPlane<float>>::getId() const
{
    return id;
}

void StaticPhysObject<GMlib::PPlane<float>>::setId(int value)
{
    id = value;
}
//**********************************************************************************************
#define end1  }

#define plane_movement {
// plane movement
void collision::StaticPhysObject<GMlib::PPlane<float> >::moveUp()
{
    GMlib::Vector<float,3> dS =  GMlib::Vector<float,3>(0,0.5,0);
    this->translateGlobal(dS);
}

void collision::StaticPhysObject<GMlib::PPlane<float> >::moveDown()
{
    GMlib::Vector<float,3> dS =  GMlib::Vector<float,3>(0,-0.5,0);
    this->translateGlobal(dS);
}

void collision::StaticPhysObject<GMlib::PPlane<float> >::moveLeft()
{
    GMlib::Vector<float,3> dS =  GMlib::Vector<float,3>(-0.5,0,0);
    this->translateGlobal(dS);
}

void collision::StaticPhysObject<GMlib::PPlane<float> >::moveRight()
{
    GMlib::Vector<float,3> dS =  GMlib::Vector<float,3>(0.5,0,0);
    this->translateGlobal(dS);
}
//**********************************************************************************************
#define end1  }

// connect tests
std::unique_ptr<Controller> unittestCollisionControllerFactory() {
    return std::make_unique<MyController> ();
}

/** Method developed with help of Bjørn-Richard Pedersen and Fatemeh Heidari **/
// check the planes against a sphere and find a closest one
GMlib::Vector<float,3> MyController::getClosestPoint(DynamicPSphere* sphere, seconds_type dt)
{
    //going to return   q-p  to  adjustTrajectory method
    auto max_dt = dt;
    auto min_dt = sphere->curr_t_in_dt;
    auto new_dt = max_dt -min_dt;

    float u = 0.4;
    float v = 0.4;
    float delta_u = 0.5;
    float delta_v = 0.5;

    auto s = sphere->getMatrixToScene() * sphere->getPos();
    GMlib::Vector<double, 3> ds = sphere->computeTrajectory(new_dt);
    auto p = s+ds;
    GMlib::SqMatrix<float,2> A;
    GMlib::Vector<float, 2> b;
    GMlib::Vector<float,3> d{0.0f,0.0f,0.0f};

    auto  planes = _map[sphere];

    //use taylor expansion
    //iteration
    for ( int i=0; i<10;i++/* delta_u > epsilon && delta_v > epsilon*/){
        GMlib::Vector <float,3>Sn {0.0f,0.0f,0.0f};
        GMlib::Vector <float,3>Su {0.0f,0.0f,0.0f};
        GMlib::Vector <float,3>Sv {0.0f,0.0f,0.0f};
        GMlib::Vector <float,3>Suu {0.0f,0.0f,0.0f};
        GMlib::Vector <float,3>Svv {0.0f,0.0f,0.0f};
        GMlib::Vector <float,3>Suv {0.0f,0.0f,0.0f};
        GMlib::APoint <float,3>q;

        for(auto it = planes.begin(); it != planes.end(); it++){
            GMlib::Vector<float,3> normal{0.0f,0.0f,0.0f};
            GMlib::DMatrix<GMlib::Vector<float,3>>  M = (*it)->evaluateParent(u,v,2,2);
            q   = M(0)(0);
            Su += M(1)(0);
            Sv += M(0)(1);
            Suu+= M(2)(0);
            Svv+= M(0)(2);
            Suv+= M(1)(1);
        }
        d =(q - p);
        A[0][0] = d* Suu + Su * Su;
        A[0][1] = d* Suv + Su * Sv;
        A[1][0] = d* Suv + Su * Sv;
        A[1][1] = d* Svv + Sv * Sv;

        GMlib::SqMatrix<float,2> A_inv = A;
        A_inv.invert();

        b[0] = - d * Su;
        b[1] = - d * Sv;

        GMlib::APoint<float, 3> X = A_inv * b;
        delta_u = X(0);
        delta_v = X(1);

        u += delta_u;
        v += delta_v;

    }
    return d;
}

/** Method developed with help of Bjørn-Richard Pedersen and Fatemeh Heidari **/
void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t){

    if( this->_state == DynamicPSphere::States::AtRest or this->velocity <= 0.04 ) {

        this->velocity = {0.0f, 0.0f, 0.0f };
        this->environment = &_sphereController->_stillEnvironment;
    }
    else {

        //move
        auto dt0 = seconds_type(t - this->curr_t_in_dt);
        auto Mi = this->getMatrixToSceneInverse();

        GMlib::Vector<double,3>  ds = (0.0f,0.0f,0.0f);

        if( this->_state == DynamicPSphere::States::Rolling )
            ds = adjustedTrajectory(dt0);
        else
            ds = computeTrajectory(dt0);

        // Move
        this->translateParent(Mi*ds);
        this->curr_t_in_dt =t;

        //update physics
        auto F = this->externalForces();
        auto c = dt0.count();
        auto a = F*c;
        this->velocity += a;
    }

}

GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::computeTrajectory(seconds_type dt) const {

    auto vel = this->velocity;
    auto dtCount = dt.count();  // 0
    auto xF = this->externalForces();
    GMlib::Vector<double,3> ds = vel * dtCount + 0.5 * xF * std::pow(dtCount, 2);

    return ds;
}

/** Method developed with help of Bjørn-Richard Pedersen and Fatemeh Heidari **/
GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::adjustedTrajectory(seconds_type dt) {

    // Update ds for modified DS
    auto ds = this->computeTrajectory(seconds_type(dt));
    auto r = this->getRadius();
    auto s = this->getMatrixToScene() * this->getPos();
    auto p = ds + s;

    auto planes = this->_sphereController->getAttachedObjects(this);
    GMlib::Vector<float,3> n {0.0f, 0.0f, 0.0f};

    GMlib::Point<float,2> q;

    for ( auto& plane : planes ) {
        const auto M = plane->evaluateParent(0.5f, 0.5f, 1, 1);
        const auto u = M(1)(0);
        const auto v = M(0)(1);
        auto normal = GMlib::Vector<float,3> (u ^ v);
        n += normal;
    }

    n = GMlib::Vector<float,3> (n / planes.size()).getNormalized();
    auto closest = this->_sphereController->getClosestPoint(this,dt);
    auto d = (n * r) + closest;

    auto adjusted_ds = ds + d;

    return adjusted_ds;

}

GMlib::Vector<double,3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const {
    assert(environment != nullptr);
    return this->environment->externalForces().toType<double>();
}

/** Method developed with help of Bjørn-Richard Pedersen and Fatemeh Heidari **/
void MyController::localSimulate(double dt) {

    // Reset time variable for all objects
    for( auto sphere : _dynamic_spheres)
        sphere->curr_t_in_dt = seconds_type{0.0};

    // Detect state changes and fill up our state container
    detectStateChanges(dt);
    sortAndMakeUniqueStates(_singularities);

    // Collision detection algorithm
    for( auto& sphere : _dynamic_spheres)
        dynamicCollision(sphere, seconds_type(dt));

    // Make Collision unique
    sortAndMakeUnique(_collisions);

    // Make both collisions and states unique in relation to each other
    if( !_collisions.empty() and !_singularities.empty() )
        crossUnique(_collisions, _singularities);
    else {
        // Make sure that the newest event is at the front of the vector
        std::reverse(_singularities.begin(), _singularities.end() );
        std::reverse(_collisions.begin(), _collisions.end());
    }


    while( !_collisions.empty() or !_singularities.empty() ) {

        // If both containers not empty
        if( !_collisions.empty() and !_singularities.empty() ) {

            const auto col_time = _collisions.back().t_in_dt;
            const auto sing_time = _singularities.back().time;

            // Resolve Collision
            if ( col_time < sing_time ) {
                auto c = _collisions.back(); //take a first collision object
                _collisions.pop_back();;     //remove it from vector

                handleCollision(c, dt);     // Also detects more collisions
            }

            // Resolve Singularity
            else {
                auto s = _singularities.back();
                _singularities.pop_back();

                handleStates(s, dt);

                // Collision detection algorithm
                for( auto& sphere : _dynamic_spheres)
                    dynamicCollision(sphere, seconds_type(dt));
            }
        }

        // IF COLLISIONS NOT EMPTY
        else if( !_collisions.empty() and _singularities.empty() ) {
            auto c = _collisions.back();
            _collisions.pop_back();;

            handleCollision(c, dt);     // Also detects more collisions

        }

        //  If singularities container not empty
        else if( _collisions.empty() and !_singularities.empty() ) {
            auto s = _singularities.back();
            _singularities.pop_back();

            handleStates(s, dt);

            // Collision detection algorithm
            for( auto& sphere : _dynamic_spheres)
                dynamicCollision(sphere, seconds_type(dt));

        }



        detectStateChanges(dt);  //detect states changes
        sortAndMakeUnique(_collisions);
        sortAndMakeUniqueStates(_singularities);

        if( !_collisions.empty() and !_singularities.empty() )
            crossUnique(_collisions, _singularities);
        else {
            // Make sure that the newest event is at the front of the vector
            std::reverse(_singularities.begin(), _singularities.end() );
            std::reverse(_collisions.begin(), _collisions.end());
        }
    }

    //  Start simulation for all objects
    for( auto sphere : _dynamic_spheres)
        sphere->simulateToTInDt(seconds_type(dt));

}

/** Method developed with help of Bjørn-Richard Pedersen and Fatemeh Heidari **/
// Singularity handeling
void MyController::handleStates(StateChangeObj &state, double dt) {

    auto sphere = state.obj;
    auto newState = state.state;
    auto Statetime = state.time;
    auto planes = state.attachedPlanes;

    std::cout << "handleStates says the state is now " << int(newState) << " after being " << int(sphere->_state) << std::endl;

    if( newState == DynamicPSphere::States::Free ) {
        // Remove objects from the set In the map (if not mistaken, check)
        _map.erase(sphere);
    }

    else {
        // Set objects attached to sphere
        for( auto& p : planes) {
            _map[sphere].emplace(p);
        }
    }
    sphere->_state = newState;
    sphere->simulateToTInDt(Statetime);
}

// Collision handeling
void MyController::handleCollision(CollisionObject &c, double dt) {

    // Add more objects here if you end up using more
    auto d_sphere_1     = dynamic_cast<DynamicPSphere*>(c.obj1);
    auto d_sphere_2     = dynamic_cast<DynamicPSphere*>(c.obj2);
    auto s_plane_2      = dynamic_cast<StaticPPlane*>(c.obj2);

    // Impact response
    // If the first object is a sphere
    if(d_sphere_1) {


        if (d_sphere_2) {

            if( d_sphere_2->_state == DynamicPSphere::States::AtRest) {

                d_sphere_1->simulateToTInDt(c.t_in_dt);
                d_sphere_2->curr_t_in_dt = d_sphere_1->curr_t_in_dt;
                d_sphere_2->environment = &_env;
                d_sphere_2->_state = DynamicPSphere::States::Rolling;

                collision::computeImpactResponse( *d_sphere_1, *d_sphere_2, c.t_in_dt);    // D_Sphere vs. D_Sphere
            }
            else {

                d_sphere_1->simulateToTInDt(c.t_in_dt);
                d_sphere_2->simulateToTInDt(c.t_in_dt);

                collision::computeImpactResponse( *d_sphere_1, *d_sphere_2, c.t_in_dt);    // D_Sphere vs. D_Sphere

            }

        }
        else if (d_sphere_1 && s_plane_2) {

            if(d_sphere_1->_state != DynamicPSphere::States::AtRest)
            {
                d_sphere_1->simulateToTInDt(c.t_in_dt);
            }
            collision::computeImpactResponse( *d_sphere_1, *s_plane_2, c.t_in_dt);     // D_Sphere vs. S_Plane
        }


    }

    // Additional collisions for the dynamic objects in the collision_object
    // Not allowed: Same collision twice, cannot collide with itself

    // If the dynamic object (obj1) is a sphere
    if( d_sphere_1) {

        dynamicCollision(d_sphere_1, seconds_type(dt));   // Does it collide with any dynamic objects? Can't with same obj as last time

        // If sphere 1 collided with a dynamic sphere, check for that sphere's future collisions
        if(d_sphere_2)

            dynamicCollision(d_sphere_2, seconds_type(dt));   // Does it collide with any dynamic objects? Can't with sphere 1


    }
}

void MyController::detectStateChanges(double dt) {

    // predicate here to check if allready in _singularities

    for( auto& sphere : _dynamic_spheres) {

        auto singularity = detectStateChange(sphere, dt);

        if (singularity.state != sphere->_state) {

            _singularities.push_back(singularity);
        }
    }

}

/** Method developed with help of Bjørn-Richard Pedersen, Ghada Bouzidi and Fatemeh Heidari **/
StateChangeObj MyController::detectStateChange(DynamicPSphere *sphere, double dt) {

    std::unordered_set<StaticPPlane*> planeContainer;       // Used for returning planes that the sphere is (not) attached to
    DynamicPSphere::States state;                           // Holder variable for the state the sphere will enter

    const auto epsilon = 1e-5;

    // Sphere variables
    const auto r = sphere->getRadius();
    const auto pos = sphere->getMatrixToScene() * sphere->getPos();

    // Time variables
    const auto sphereTime = sphere->curr_t_in_dt;
    const auto maxDt = seconds_type(dt);
    const auto newDt = maxDt - sphereTime;
    seconds_type returnTime = sphereTime;

    const auto ds = sphere->computeTrajectory(newDt);   // Calculating "original ds"

    // Plane variables.
    auto planes = getAttachedObjects(sphere);
    GMlib::APoint<float,3> q;
    GMlib::Vector<float,3> n {0.0f, 0.0f, 0.0f};

    if( planes.empty() ) {  // sphere is not attached to plane

        for (auto& plane : _static_planes) {
            auto M = plane->evaluateParent(0.5f,0.5f,1,1);
            auto q = M(0)(0);
            auto u = M(1)(0);
            auto v = M(0)(1);
            auto n = GMlib::Vector<float,3>(u ^ v).getNormalized();
            auto d = (q + r * n) - pos;

            auto bla        = std::abs(((-n*r) * ds) -(ds*ds));
            auto dsn        = ds * n;
            auto dn         = d*n;

            if( std::abs(dn) < epsilon and dsn <= epsilon ) {

                planeContainer.insert(plane);
                state = DynamicPSphere::States::Rolling;

            }
            else if( std::abs(dn) < epsilon and bla < epsilon  ) {

                planeContainer.insert(plane);
                state = DynamicPSphere::States::AtRest;

            }
            else state = DynamicPSphere::States::Free;
        }

        return StateChangeObj(sphere, planeContainer, returnTime, state);
    }
    else {     // sphere is attached to plane

        for (auto &it :planes){
            auto M = it->evaluateParent(0.5f,0.5f,1,1);
            auto pos= M(0)(0);
            auto u = M(1)(0);
            auto v = M(0)(1);
            auto normal = GMlib::Vector<float,3>(u ^ v);
            n+=normal;
            q=pos;
        }
        n= GMlib::Vector <float,3>(n/planes.size()).getNormalized();

        auto d       = (q + r * n) - pos;
        auto bla     = std::abs(((-n*r) * ds) -(ds*ds));
        auto dsn     = ds * n;
        auto dn      = d * n;

        if( sphere->_state == DynamicPSphere::States::Rolling ) {

            if( std::abs(dn) > epsilon and dsn > epsilon) {

                state = DynamicPSphere::States::Free;

                auto x = dn / dsn;
                returnTime      = (x * newDt) + sphereTime;

                return StateChangeObj(sphere, planes, returnTime, state);
            }
            else if( bla < epsilon ) {

                state = DynamicPSphere::States::AtRest;

                auto x = dn / dsn;
                returnTime      = (x * newDt) + sphereTime;

                return StateChangeObj(sphere, planes, returnTime, state);
            }
            else return StateChangeObj(sphere, planes, returnTime, DynamicPSphere::States::Rolling);
        }

        else if( sphere->_state == DynamicPSphere::States::AtRest ) {

            if( bla > epsilon ) {

                state = DynamicPSphere::States::Rolling;
                return StateChangeObj(sphere, planes, returnTime, DynamicPSphere::States::Rolling);
            }
            else if( dsn > epsilon) {

                state = DynamicPSphere::States::Free;
                return StateChangeObj(sphere, planes, returnTime, DynamicPSphere::States::Rolling);
            }

            else return StateChangeObj(sphere, planes, returnTime, DynamicPSphere::States::AtRest);
        }
    }
}

// unordered_set; Get, Set and Remove objects to / from Sphere
// Get objects attached to sphere
std::unordered_set<StaticPPlane *> MyController::getAttachedObjects(DynamicPSphere* sphere)
{
    static std::unordered_set<StaticPPlane*> empty {};
    auto iter = _map.find(sphere);

    if( iter != _map.end() ) {

        return iter->second;
    }
    else return empty;
}


//void collision::removeCube(int id)
//{

//    std::cout<<id<<std::endl;

//    const GMlib::Array<GMlib::SceneObject*> &selected_objects = _scene->getSelectedObjects();

//      for( int i = 0; i < _static_planes.getSize(); i++ )
//      {
//          GMlib::SceneObject* obj = selected_objects(i);
//          _scene->remove(obj);
//      }

//}


} // END namespace collision

