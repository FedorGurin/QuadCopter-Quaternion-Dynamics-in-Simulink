#include "report.h"
#include "globalNameID.h"
#include <iomanip>

Report::Report(uint32_t idClass, const std::string &fileName_, int decim_):ICalculateElement(idClass)
{
    stringUId = "report";
    fileName  = fileName_;
    decim     = (decim_ < 1) ? 1 : decim_;
    count     = 0;
    // указываем зависимость
    bindTo(ID_OS4_Output, &output);
}
Report::~Report()
{
    if(file.is_open())
        file.close();
}
void Report::init()
{
    ICalculateElement::init();
    file.open(fileName.c_str());
    if(!file.is_open())
        return;
    file << "time,"
         << "Ve_x,Ve_y,Ve_z,Xe_x,Xe_y,Xe_z,"                 // os4sim_stateX
         << "Vb_x,Vb_y,Vb_z,p,q,r,"                          // os4sim_stateB
         << "q0,q1,q2,q3,"                                   // stateQ
         << "roll,pitch,yaw,"                                // углы Эйлера 3-2-1, рад
         << "Vinf,psi_air,theta_air,"                        // os4sim_stateA
         << "acc_x,acc_y,acc_z,"                             // os4sim_IMU (акселерометр)
         << "tau1,tau2,tau3,tau4,T1,T2,T3,T4,w1,w2,w3,w4,"   // os4sim_control
         << "hx_x,hx_y,hx_z,hy_x,hy_y,hy_z"                  // os4sim_stateHeading
         << "\n";
    file << std::setprecision(9);
    count = 0;
    writeRow();
}
void Report::calculate()
{
    count++;
    if(count >= decim)
    {
        count = 0;
        writeRow();
    }
}
void Report::writeRow()
{
    if(!file.is_open())
        return;
    const TOS4Output &o = *output;
    file << o.time << ","
         << o.Ve.x << "," << o.Ve.y << "," << o.Ve.z << ","
         << o.Xe.x << "," << o.Xe.y << "," << o.Xe.z << ","
         << o.Vb.x << "," << o.Vb.y << "," << o.Vb.z << ","
         << o.pqr.x << "," << o.pqr.y << "," << o.pqr.z << ","
         << o.q.q0 << "," << o.q.q1 << "," << o.q.q2 << "," << o.q.q3 << ","
         << o.euler.x << "," << o.euler.y << "," << o.euler.z << ","
         << o.aero.Vinf << "," << o.aero.psi << "," << o.aero.theta << ","
         << o.acc.x << "," << o.acc.y << "," << o.acc.z;
    for(int i = 0; i < OS4_NUM_ROTORS; i++) file << "," << o.tau[i];
    for(int i = 0; i < OS4_NUM_ROTORS; i++) file << "," << o.T[i];
    for(int i = 0; i < OS4_NUM_ROTORS; i++) file << "," << o.omega[i];
    file << "," << o.headingX.x << "," << o.headingX.y << "," << o.headingX.z
         << "," << o.headingY.x << "," << o.headingY.y << "," << o.headingY.z
         << "\n";
}
