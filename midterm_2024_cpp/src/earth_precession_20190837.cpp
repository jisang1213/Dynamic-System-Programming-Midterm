#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include "../include/mymat.hpp"
#include "../include/myQuaternion.hpp"

int main()
{
    std::ofstream q_results("q_results_cpp.csv");
    std::ofstream w_results("w_results_cpp.csv");

    double h = 0.001; // simulation timestep, 0.001 seconds (1 ms)
    int n_sim = 1000000; // number of simulation steps (1000 s)
    int rec_steps = 100; // record the result every n steps (every 0.1 s)
    Quaternion Q(0.97904547, 0, -0.20364175, 0); // initial orientation quaternion
    Mat33 I_lcs(120., 120., 120.);  // inertia of the body (in Earth local coordinate system)
    Vec3 w_lcs(0.0, 0.0, 10.0); // angular velocity in Earth local coordinate system
    Vec3 wd_lcs(0.0, 0.0, 0.0); // angular acceleration in Earth local coordinate system
    std::cout << "Simulation start" << std::endl;

    //////////////// TODO ////////////////
    // TODO: you can set yout own variable.

    Vec3 IxW;  //I_lcs cross w_lcs
    Vec3 T_dir, torque;
    Vec3 z_w(0,0,1);
    Vec3 z_b;
    Vec3 w_gcs;
    Mat33 R_b2w = quat2mat(Q); //DELETE
    Mat33 I_lcs_inverse = I_lcs.inverse();
    Quaternion dQ;

    for (int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        // TODO: implement semi-implicit Euler method to simulate the earth precession

        //compute the torque in the body frame
        Q.inversed();
        z_b = vecRotatedByQuat(z_w, Q); //express world z-axis in body frame
        T_dir = w_lcs.cross(z_b);
        torque = T_dir * (20.0/sqrt(T_dir.squarednorm()));

        //find rate change of angular velocity in local frame
        IxW = I_lcs*w_lcs;
        wd_lcs = I_lcs_inverse*(torque + IxW.cross(w_lcs));

        //increment angular velocity in local frame
        w_lcs = w_lcs + wd_lcs*h;

        Q.inversed();
        w_gcs = vecRotatedByQuat(w_lcs, Q);   ///CHANGE TO QUAT
        dQ = getQuaternionBetweenTimeStep(w_gcs, h);

        //Update orientation
        Q = quatmulquat(dQ,Q);
        Q.normalized(); //normalize to prevent drift


        // TODO: record q_results_cpp.csv contains earth quaternion in global coordinate system (w,x,y,z)
        // TODO: record w_results_cpp.csv contains earth angular velocity in global coordinate system (x,y,z)
        if (i_sim % rec_steps == 0)
        {
            // record the orientation (gcs)

            q_results << i_sim * h << ", " << Q.w() << ", " << Q.x() << ", " << Q.y() << ", " << Q.z() << ", " << "\n";

            // record the angular velocity (gcs)

            w_results << i_sim * h << ", " << w_gcs.get_elem(0,0) << ", " << w_gcs.get_elem(1,0) << ", " << w_gcs.get_elem(2,0) << ", " << "\n";

        }
        //////////////// TODO end ////////////////
    }
    std::cout << "Simulation end" << std::endl;

    // close the file
    q_results.close();
    w_results.close();

    return 0;
}