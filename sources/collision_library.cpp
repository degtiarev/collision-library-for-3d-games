#include "collision_library.h"



namespace collision
{

//******************* Dectection collsion states *****************************************

// dynamic sphere - dynamic sphere
CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
                                const DynamicPhysObject<GMlib::PSphere<float>>& S1,
                                seconds_type        dt)
{
    const auto dt_max = dt;
    const auto dt_min = std::max(S0.curr_t_in_dt, S1.curr_t_in_dt);
    const auto new_dt = dt_max - dt_min;

    const auto S0_position = S0.getMatrixToScene() * S0.getPos();
    const auto S1_position = S1.getMatrixToScene() * S1.getPos();

    const auto S0_radius = S0.getRadius();
    const auto S1_radius = S1.getRadius();
    const auto radius_sum = S0_radius + S1_radius;

    const auto Q = (S1_position - S0_position);
    const auto R = (S1.computeTrajectory(new_dt) - S0.computeTrajectory(new_dt));

    const auto _QR = Q * R;
    const auto _QRQR = std::pow( _QR, 2);

    const auto _RR = R * R;
    const auto _QQ = Q * Q;

    const auto _rr = std::pow( radius_sum, 2);
    const auto _square = std::sqrt(_QRQR - (_RR * (_QQ - _rr)));

    const auto epsilon = 0.00001;

    if ( _square < 0 )
    {
        return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityNoCollision);
    }
    else if ( (_QQ - _rr) < epsilon )
    {
        return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityParallelAndTouching);
    }
    else if ( _RR < epsilon )
    {
        return CollisionState( seconds_type(0.0), CollisionStateFlag::SingularityParallel);
    }

    const auto x = (-_QR - _square) / _RR;

    return CollisionState(((x * new_dt) + dt_min), CollisionStateFlag::Collision);
}

// dynamic sphere - static plane
CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S,
                                const StaticPhysObject<GMlib::PPlane<float>>&   P,
                                seconds_type                            dt)
{
    //auto unconst_S = const_cast<DynamicPhysObject<GMlib::PSphere<float>>&>(S);

    const auto dt_max = dt;
    const auto dt_min = S.curr_t_in_dt;
    const auto new_dt = dt_max - dt_min;

    auto &unconst_P = const_cast<StaticPhysObject<GMlib::PPlane<float>>&>(P);
    auto s_pos =S.getMatrixToScene()*S.getPos();
    auto _Radius = S.getRadius();
    auto plane_pos = unconst_P.evaluateParent(0.5f, 0.5f,1, 1);

    auto v = plane_pos(0)(1);
    auto u = plane_pos(1)(0);
    auto n = u^v;

    auto nNormal = GMlib::Vector<float,3>(n).getNormalized();
    auto d = (plane_pos(0)(0) + _Radius*nNormal) - s_pos;

    const auto ds = (S.computeTrajectory(new_dt)*nNormal);

    auto x = (d*nNormal)/(S.computeTrajectory(new_dt)*nNormal);

    const auto epsilon = 0.00001;

    if ( std::abs(d*n) < epsilon )
    {
        return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityParallelAndTouching);
    }
    else if (std::abs(ds)< epsilon )
    {
        return CollisionState( seconds_type(0.0), CollisionStateFlag::SingularityParallel);
    }

    return CollisionState(((x*new_dt)+ dt_min),CollisionStateFlag::Collision);

}

// dynamic sphere - static sphere
CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
                                const StaticPhysObject<GMlib::PSphere<float>>&  S1,
                                seconds_type                                    dt)
{
}

// dynamic sphere - static cylinder
CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>&  S,
                                const StaticPhysObject<GMlib::PCylinder<float>>& C,
                                seconds_type                                    dt)
{
}

// dynamic sphere - static Bezier
CollisionState detectCollision (const DynamicPSphere&  S,
                                const StaticPBezierSurf& B, seconds_type dt)
{
    const auto dt_max = dt;
    const auto dt_min = S.curr_t_in_dt;
    const auto new_dt = dt_max - dt_min;

    const auto _Radius = S.getRadius();//r
    const auto s_pos = S.getPos();//p

    float u,v,t;
    t = 0.0;
    u = 0.5;
    v = 0.5;
    const auto epsilon = 1e-5;

    for(int i=0; i<6; i++){

        auto ds = S.computeTrajectory(new_dt);
        auto &unconst_B = const_cast<StaticPBezierSurf&>(B);
        const auto surf_pos = unconst_B.evaluate(u, v,1, 1);

        const auto s_pos_new = s_pos + ds*t;
        const auto q = surf_pos(0)(0);
        const auto Sv = surf_pos(0)(1);
        const auto Su = surf_pos(1)(0);

        //const auto ds_new = ds*t;

        const auto Sn = GMlib::Vector<float,3>(Su ^ Sv).getNormalized();

        GMlib::SqMatrix<float,3>A;
        A.setCol(Su,0);
        A.setCol(Sv,1);
        A.setCol(-ds,2);
        auto A_inv = A;

        A_inv.invert();

        const auto b = GMlib::Vector<float,3>{s_pos_new-q-Sn*_Radius};
        const auto x = A_inv*b;

        const auto deltaU = x(0);
        const auto deltaV = x(1);
        const auto deltaT = x(2);

        u+=deltaU;
        v+=deltaV;
        t+=deltaT;

        if( (std::abs(deltaU) < epsilon) and (std::abs(deltaV) < epsilon) and (std::abs(deltaT) < epsilon) ) {

            return CollisionState(seconds_type(deltaT), CollisionStateFlag::Collision);
        }
    }

    return CollisionState(seconds_type(dt_min), CollisionStateFlag::SingularityNoCollision);
}

// dynamic sphere - static torus
CollisionState detectCollision (const DynamicPSphere&  S,
                                const StaticPTorus& T, seconds_type dt)
{

}

//***************************************************************************************


//******************* Compututation response ********************************************

// dynamic sphere - dynamic sphere
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& sphere0,
                            DynamicPhysObject<GMlib::PSphere<float>>& sphere1,
                            seconds_type                              dt)
{
    auto s0_pos = sphere0.getPos().toType<double>();
    auto s1_pos = sphere1.getPos().toType<double>();
    auto d = (s1_pos - s0_pos);
    auto dNormalized = GMlib::Vector<double,3>(d).getNormalized();
    auto vel0 = sphere0.velocity;
    auto vel1 = sphere1.velocity;
    auto n = GMlib::Vector<double,3>(d).getLinIndVec();
    auto nNormal = n.getNormalized();
    auto V0d = (vel0*dNormalized);
    auto V1d = (vel1*dNormalized);
    auto V0n = (vel0*nNormal);
    auto V1n = (vel1*nNormal);
    auto S0mass = sphere0.mass;
    auto S1mass = sphere1.mass;
    auto V0d_new = (((S0mass-S1mass)/(S0mass+S1mass))*V0d + ((2*S1mass)/(S0mass+S1mass))*V1d);
    auto V1d_new = (((S1mass-S0mass)/(S0mass+S1mass))*V1d + ((2*S0mass)/(S0mass+S1mass))*V0d);
    sphere0.velocity = (V0n*nNormal + (V0d_new*dNormalized));
    sphere1.velocity = (V1n*nNormal + (V1d_new*dNormalized));

}

// dynamic sphere - static plane
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& sphere,
                            const StaticPhysObject<GMlib::PPlane<float>>&   plane,
                            seconds_type                              dt)
{

    auto &unconst_P = const_cast<StaticPhysObject<GMlib::PPlane<float>>&>(plane);
    const auto s_pos =sphere.getPos();
    const auto _Radius = sphere.getRadius();
    auto plane_pos = unconst_P.evaluateParent(0.5f, 0.5f, 1, 1);
    const auto v = plane_pos(0)(1);
    const auto u = plane_pos(1)(0);
    const auto n = u^v;
    const auto nNormal = GMlib::Vector<float,3>(n).getNormalized();

    auto newvelocity= (sphere.velocity-2.0f*(sphere.velocity*nNormal)*nNormal);
    sphere.velocity=newvelocity;

}

// dynamic sphere - static sphere
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
                            StaticPhysObject<GMlib::PSphere<float>>&  S1,
                            seconds_type                              dt)
{
}

// dynamic sphere - static cylinder
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>&  S,
                            StaticPhysObject<GMlib::PCylinder<float>>& C,
                            seconds_type                              dt)
{
}

// dynamic sphere - static torus
void computeImpactResponse (DynamicPSphere& S, const StaticPTorus& T,
                            seconds_type dt)
{

}

//***************************************************************************************


void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t)
{
    //start
    auto const M = this->getMatrixToSceneInverse();
    auto dt = this->curr_t_in_dt;
    const auto new_dt = t-dt;

    auto ds = this->computeTrajectory(new_dt);
    // auto xds = ds*new_dt.count();

    //move
    this->translateParent(M*ds);
    //physics
    auto F = this->externalForces();
    auto a = 0.5*F*std::pow(dt.count(),2)*this->mass;
    this->velocity+=a;

}

GMlib::Vector<double, 3> DynamicPhysObject<GMlib::PSphere<float> >::computeTrajectory(seconds_type dt) const
{
    return this->velocity*dt.count();
}

GMlib::Vector<double, 3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const
{
    assert(environment != nullptr);
    environment->externalForces();
}

void MyController::localSimulate(double dt)
{
    for (auto sphere:_dynamic_spheres)
    {
        sphere->curr_t_in_dt=seconds_type(0);
    }


    collisionAlgorithm (seconds_type (dt));
    //add collsion
    for (auto sphere:_dynamic_spheres)
    {
        sphere->simulateToTInDt(seconds_type (dt));

    }

}

void MyController::collisionAlgorithm(seconds_type dt)
{
    std::vector<CollisionObject> C;
    for (auto &sphere:_dynamic_spheres){

        for (auto &plane:_static_planes){

            auto dt_min = sphere->curr_t_in_dt;

            const auto state = detectCollision(*sphere,*plane,dt);

            if (state.flag == CollisionStateFlag::Collision and (state.time>dt_min && state.time<=dt))
                C.push_back(CollisionObject(sphere,plane,state.time));
        }
    }
    sortAndMakeUnique(C);
    std::reverse(C.begin(),C.end());
    while (!(C.empty())){
        auto c_elem = C.back();
        C.pop_back();
        c_elem.obj1->simulateToTInDt(dt);

        auto obj1_dsphere = dynamic_cast<DynamicPSphere*>(c_elem.obj1);
        //            auto obj1_dcylinder = dynamic_cast<DynamicPSphere*>(c_elem.obj1);

        // auto obj2_dsphere = dynamic_cast<DynamicPSphere*>(c_elem.obj2);
        auto obj2_splane = dynamic_cast<const StaticPPlane*>(c_elem.obj2);

        if(obj1_dsphere and obj2_splane)
            computeImpactResponse(*obj1_dsphere,*obj2_splane,c_elem.t_in_dt);
        //            else if(obj1_dsphere and obj2_dsphere)
        //                computeImpactResponse(obj1_dsphere,obj2_dsphere,dt);



    }
}

std::unique_ptr<Controller> unittestCollisionControllerFactory()
{

    return std::make_unique<MyController>();
}

void MyController::add(DynamicPSphere * const sphere)
{
    sphere->environment = & _enviroment;
    _dynamic_spheres.push_back(sphere);
}

void MyController::add(StaticPPlane * const plane) { _static_planes.push_back(plane); }

void MyController::add (StaticPSphere* const sphere) { _static_spheres.push_back(sphere); }

void MyController::add(StaticPCylinder * const cylinder) { _static_cylinders.push_back(cylinder); }

void MyController::add(StaticPBezierSurf * const surf) { _static_bezier_surf.push_back(surf); }



} // END namespace collision

