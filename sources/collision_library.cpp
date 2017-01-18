#include "collision_library.h"



namespace collision
{


//    CollisionState
//    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
//                     const DynamicPhysObject<GMlib::PSphere<float>>& S1,
//                     seconds_type                                    dt)
//    {
//    }

//    CollisionState
//    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S0,
//                     const StaticPhysObject<GMlib::PSphere<float>>&  S1,
//                     seconds_type                                    dt)
//    {
//    }

//    CollisionState
//    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& S,
//                     const StaticPhysObject<GMlib::PPlane<float>>&   P,
//                     seconds_type                                    dt)
//    {
//    }

//    CollisionState
//    detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>&  S,
//                     const StaticPhysObject<GMlib::PCylinder<float>>& C,
//                     seconds_type                                    dt)
//    {
//    }



//    void
//    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
//                           DynamicPhysObject<GMlib::PSphere<float>>& S1,
//                           seconds_type                              dt)
//    {
//    }

//    void
//    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S0,
//                           StaticPhysObject<GMlib::PSphere<float>>&  S1,
//                           seconds_type                              dt)
//    {
//    }

//    void
//    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& S,
//                           StaticPhysObject<GMlib::PPlane<float>>&   P,
//                           seconds_type                              dt)
//    {
//    }

//    void
//    computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>&  S,
//                           StaticPhysObject<GMlib::PCylinder<float>>& C,
//                           seconds_type                              dt)
//    {
//    }

CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& sphere0,
                                const DynamicPhysObject<GMlib::PSphere<float>>& sphere1,
                                seconds_type                                    dt)
{




    const auto S0_pose = sphere0.getPos();
    const auto S1_pose = sphere1.getPos();
    const auto S0_rad = sphere0.getRadius();
    const auto S1_rad = sphere1.getRadius();
    const auto rad_sum = S0_rad + S1_rad;
    const auto Q = S1_pose - S0_pose;
    const auto R = sphere1.computeTrajectory(dt) - sphere0.computeTrajectory(dt);
    const auto r_square = std::pow(rad_sum,2);
    const auto x = (-(Q*R) - sqrt(std::pow(Q*R,2) - (R*R)*((Q*Q)-r_square)))/(R*R);
    auto d = Q + x*R;
    std::cout << x << std::endl;

    //    auto tmin=std:max();



    return CollisionState(x*dt);

}

CollisionState detectCollision (const DynamicPhysObject<GMlib::PSphere<float>>& sphere,
                                const StaticPhysObject<GMlib::PPlane<float>>&   plane,
                                seconds_type                                    dt)
{

    auto &unconst_P = const_cast<StaticPhysObject<GMlib::PPlane<float>>&>(plane);
    const auto s_pos =sphere.getPos();
    const auto _Radius = sphere.getRadius();
    auto plane_pos = unconst_P.evaluateParent(0.5f, 0.5f, 1, 1);
    const auto v = plane_pos(0)(1);
    const auto u = plane_pos(1)(0);
    const auto n = u^v;
    const auto nNormal = GMlib::Vector<float,3>(n).getNormalized();
    const auto d = (plane_pos(0)(0) + _Radius*nNormal) - s_pos;
    const auto x = (d*nNormal)/(sphere.computeTrajectory(dt)*nNormal);

    return CollisionState(x*dt);

}


void
computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& sphere0,
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

void
computeImpactResponse (DynamicPhysObject<GMlib::PSphere<float>>& sphere,
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

void DynamicPhysObject<GMlib::PSphere<float> >::simulateToTInDt(seconds_type t)
{

}

GMlib::Vector<float, 3> DynamicPhysObject<GMlib::PSphere<float> >::computeTrajectory(seconds_type dt) const
{

}

GMlib::Vector<float, 3> DynamicPhysObject<GMlib::PSphere<float> >::externalForces() const
{
    assert(environment != nullptr);
    environment->externalForces();
}

std::unique_ptr<Controller> unittestCollisionControllerFactory(){

         return std::make_unique<MyController>();
}


} // END namespace collision

