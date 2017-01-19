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
                                const StaticPBezierSurf& B,
                                seconds_type dt)
{

    return CollisionState(seconds_type(0.0), CollisionStateFlag::SingularityNoCollision);

}

//***************************************************************************************


//******************* Compututation response ********************************************

// dynamic sphere - dynamic sphere
void computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& sphere0,
                            DynamicPhysObject<GMlib::PSphere<float>>& sphere1,
                            seconds_type                              dt)
{
    auto s0_pos = sphere0.getPos();
    auto s1_pos = sphere1.getPos();
    auto d = (s1_pos - s0_pos);
    auto dNormalized = GMlib::Vector<float,3>(d).getNormalized();
    auto vel0 = sphere0.velocity;
    auto vel1 = sphere1.velocity;
    auto n = GMlib::Vector<float,3>(d).getLinIndVec();
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

//***************************************************************************************


void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t)
{

}

GMlib::Vector<float, 3> DynamicPhysObject<GMlib::PSphere<float> >::computeTrajectory(seconds_type dt) const
{
    return this->velocity*dt.count();
}

GMlib::Vector<float, 3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const
{
    assert(environment != nullptr);
    environment->externalForces();
}

std::unique_ptr<Controller> unittestCollisionControllerFactory()
{

    return std::make_unique<MyController>();
}



} // END namespace collision

