#include "cbshot.h"


void cbshot::calculate_ICP_COV(pcl::PointCloud<pcl::PointNormal>& data_pi, pcl::PointCloud<pcl::PointNormal>& model_qi, Eigen::Matrix4f& transform, Eigen::MatrixXd& ICP_COV)
{

    double Tx = transform(0,3);
    double Ty = transform(1,3);
    double Tz = transform(2,3);
    double roll  = atan2f(transform(2,1), transform(2,2));
    double pitch = asinf(-transform(2,0));
    double yaw   = atan2f(transform(1,0), transform(0,0));

    double x, y, z, a, b, c;
    x = Tx; y = Ty; z = Tz;
    a = yaw; b = pitch; c = roll;// important // According to the rotation matrix I used and after verification, it is Yaw Pitch ROLL = [a,b,c]== [R] matrix used in the MatLab also :)

    /* Flushing out in the form of XYZ ABC */
    std::cout << "\nPrinting out [x, y, z, a, b, c] =  " <<x<<"    "<<y<<"    "<<z<<"    "<<a<<"    "<<b<<"    "<<c<<std::endl;

    //Matrix initialization
    Eigen::MatrixXd d2J_dX2(6,6);
    d2J_dX2 = Eigen::MatrixXd::Zero(6,6);

    /****  Calculating d2J_dX2  ****/
    for (size_t s = 0; s < data_pi.points.size(); ++s )
    {
        double pix = data_pi.points[s].x;
        double piy = data_pi.points[s].y;
        double piz = data_pi.points[s].z;
        double qix = model_qi.points[s].x;
        double qiy = model_qi.points[s].y;
        double qiz = model_qi.points[s].z;

        double nix = model_qi[s].normal_x;
        double niy = model_qi[s].normal_y;
        double niz = model_qi[s].normal_z;

        if (niz!=niz)// for nan removal in the input point cloud data:)
            continue;

        /***********************************************************************

d2J_dX2 -- X is the [R|T] in the form of [x, y, z, a, b, c]
x, y, z is the translation part 
a, b, c is the rotation part in Euler format
[x, y, z, a, b, c] is acquired from the Transformation Matrix returned by ICP.

Now d2J_dX2 is a 6x6 matrix of the form

d2J_dx2                   
d2J_dxdy    d2J_dy2
d2J_dxdz    d2J_dydz    d2J_dz2
d2J_dxda    d2J_dyda    d2J_dzda   d2J_da2
d2J_dxdb    d2J_dydb    d2J_dzdb   d2J_dadb   d2J_db2
d2J_dxdc    d2J_dydc    d2J_dzdc   d2J_dadc   d2J_dbdc   d2J_dc2

*************************************************************************/



        double 	d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dcdx,
                d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;

        // These terms are generated from the provided Matlab scipts. We just have to copy
        // the expressions from the matlab output with two very simple changes.
        // The first one being the the sqaure of a number 'a' is shown as a^2 in matlab,
        // which is converted to pow(a,2) in the below expressions.
        // The second change is to add ';' at the end of each expression :)
        // In this way, matlab can be used to generate these terms for various objective functions of ICP
        // and they can simply be copied to the C++ files and with appropriate changes to ICP estimation,
        // its covariance can be easily estimated.


        d2J_dx2 =

                2*pow(nix,2);


        d2J_dy2 =

                2*pow(niy,2);


        d2J_dz2 =

                2*pow(niz,2);


        d2J_dydx =

                2*nix*niy;


        d2J_dxdy =

                2*nix*niy;


        d2J_dzdx =

                2*nix*niz;


        d2J_dxdz =

                2*nix*niz;


        d2J_dydz =

                2*niy*niz;


        d2J_dzdy =

                2*niy*niz;


        d2J_da2 =

                (niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a))) - (2*nix*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) + 2*niy*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_db2 =

                (niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))) - (2*niy*(pix*cos(b)*sin(a) + piz*cos(c)*sin(a)*sin(b) + piy*sin(a)*sin(b)*sin(c)) + 2*niz*(piz*cos(b)*cos(c) - pix*sin(b) + piy*cos(b)*sin(c)) + 2*nix*(pix*cos(a)*cos(b) + piz*cos(a)*cos(c)*sin(b) + piy*cos(a)*sin(b)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dc2 =

                (nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)))*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c))) - (2*niy*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b))) - 2*nix*(piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) - piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b))) + 2*niz*(piz*cos(b)*cos(c) + piy*cos(b)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));

        d2J_dxda =

                nix*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dadx =

                2*nix*(niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dyda =

                niy*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dady =

                2*niy*(niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dzda =

                niz*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dadz =

                2*niz*(niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dxdb =

                nix*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dbdx =

                2*nix*(niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dydb =

                niy*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dbdy =

                2*niy*(niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));

        d2J_dzdb =

                niz*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dbdz =

                2*niz*(niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dxdc =

                nix*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dcdx =

                2*nix*(nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dydc =

                niy*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));

        d2J_dcdy =

                2*niy*(nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dzdc =

                niz*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dcdz =

                2*niz*(nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dadb =

                (niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))) - (2*nix*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niy*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dbda =

                (2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c))) - (2*nix*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niy*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dbdc =

                (2*nix*(piy*cos(a)*cos(b)*cos(c) - piz*cos(a)*cos(b)*sin(c)) - 2*niz*(piy*cos(c)*sin(b) - piz*sin(b)*sin(c)) + 2*niy*(piy*cos(b)*cos(c)*sin(a) - piz*cos(b)*sin(a)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c))) + (niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dcdb =

                (2*nix*(piy*cos(a)*cos(b)*cos(c) - piz*cos(a)*cos(b)*sin(c)) - 2*niz*(piy*cos(c)*sin(b) - piz*sin(b)*sin(c)) + 2*niy*(piy*cos(b)*cos(c)*sin(a) - piz*cos(b)*sin(a)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c))) + (2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dcda =

                (2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c))) + (2*nix*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niy*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dadc =

                (niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c))) + (2*nix*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niy*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));





        Eigen::MatrixXd d2J_dX2_temp(6,6);

        d2J_dX2_temp << d2J_dx2,     d2J_dydx,	  d2J_dzdx,   d2J_dadx,   d2J_dbdx,     d2J_dzdx,
                        d2J_dxdy,    d2J_dy2,	  d2J_dzdy,   d2J_dady,   d2J_dbdy,     d2J_dcdy,
                        d2J_dxdz,    d2J_dydz,    d2J_dz2,    d2J_dadz,   d2J_dbdz,     d2J_dcdz,
                        d2J_dxda,    d2J_dyda,    d2J_dzda,   d2J_da2,	  d2J_dbda,     d2J_dcda,
                        d2J_dxdb,    d2J_dydb,    d2J_dzdb,   d2J_dadb,   d2J_db2,      d2J_dcdb,
                        d2J_dxdc,    d2J_dydc,    d2J_dzdc,   d2J_dadc,   d2J_dbdc,     d2J_dc2;


        d2J_dX2 = d2J_dX2 + d2J_dX2_temp;

    }// End of the FOR loop!!!

    std::cout << "\n**************\n Successfully Computed d2J_dX2 \n**************\n" << std::endl;



    // Now its time to calculate d2J_dZdX , where Z are the measurements Pi and Qi, X = [x,y,z,a,b,c]

    // n is the number of correspondences
    int n = data_pi.points.size();





    /*  Here we check if the number of correspondences between the source and the target point clouds are greater than 200.
      if yes, we only take the first 200 correspondences to calculate the covariance matrix
      You can try increasing it but if its too high, the system may run out of memory and give an exception saying

     terminate called after throwing an instance of 'std::bad_alloc'
     what():  std::bad_alloc
     Aborted (core dumped)

  */
    if (n > 200) n = 200;////////////****************************IMPORTANT CHANGE***** but may not affect********************/////////////////////////////////////////

    std::cout << "\nNumber of Correspondences used for ICP's covariance estimation = " << n << std::endl;

    Eigen::MatrixXd d2J_dZdX(6,6*n);

    for (int k = 0; k < n ; ++k) // row
    {
        //here the current correspondences are loaded into Pi and Qi
        double pix = data_pi.points[k].x;
        double piy = data_pi.points[k].y;
        double piz = data_pi.points[k].z;
        double qix = model_qi.points[k].x;
        double qiy = model_qi.points[k].y;
        double qiz = model_qi.points[k].z;

        double nix = model_qi[k].normal_x;
        double niy = model_qi[k].normal_y;
        double niz = model_qi[k].normal_z;

        if (niz!=niz) // for nan removal in input point cloud data
            continue;

        Eigen::MatrixXd d2J_dZdX_temp(6,6);


        double 	d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                d2J_dpix_dz,    d2J_dpiy_dz,    d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                d2J_dpix_da,    d2J_dpiy_da,    d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                d2J_dpix_db,    d2J_dpiy_db,    d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                d2J_dpix_dc,    d2J_dpiy_dc,    d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;


        d2J_dpix_dx =

                2*nix*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a));


        d2J_dpix_dy =

                2*niy*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a));


        d2J_dpix_dz =

                2*niz*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a));


        d2J_dpix_da =

                (2*niy*cos(a)*cos(b) - 2*nix*cos(b)*sin(a))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c))) + (2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a));


        d2J_dpix_db =

                (2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a)) - (2*niz*cos(b) + 2*nix*cos(a)*sin(b) + 2*niy*sin(a)*sin(b))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpix_dc =

                (2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)))*(nix*cos(a)*cos(b) - niz*sin(b) + niy*cos(b)*sin(a));


        d2J_dpiy_dx =

                2*nix*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c));


        d2J_dpiy_dy =

                2*niy*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c));


        d2J_dpiy_dz =

                2*niz*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c));


        d2J_dpiy_da =

                (2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c)) - (2*nix*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) + 2*niy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpiy_db =

                (2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c)) + (2*nix*cos(a)*cos(b)*sin(c) - 2*niz*sin(b)*sin(c) + 2*niy*cos(b)*sin(a)*sin(c))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpiy_dc =

                (2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)))*(niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + niz*cos(b)*sin(c)) + (2*nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - 2*niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*niz*cos(b)*cos(c))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpiz_dx =

                2*nix*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c));


        d2J_dpiz_dy =

                2*niy*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c));


        d2J_dpiz_dz =

                2*niz*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c));


        d2J_dpiz_da =

                (2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)))*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c)) + (2*nix*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + 2*niy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpiz_db =

                (2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)))*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c)) + (2*nix*cos(a)*cos(b)*cos(c) - 2*niz*cos(c)*sin(b) + 2*niy*cos(b)*cos(c)*sin(a))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dpiz_dc =

                (2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)))*(nix*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - niy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + niz*cos(b)*cos(c)) - (2*niy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - 2*nix*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + 2*niz*cos(b)*sin(c))*(nix*(x - qix - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + pix*cos(a)*cos(b)) + niy*(y - qiy + piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)) + niz*(z - qiz - pix*sin(b) + piz*cos(b)*cos(c) + piy*cos(b)*sin(c)));


        d2J_dqix_dx =

                -2*pow(nix,2);


        d2J_dqix_dy =

                -2*nix*niy;


        d2J_dqix_dz =

                -2*nix*niz;


        d2J_dqix_da =

                -nix*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dqix_db =

                -nix*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dqix_dc =

                -nix*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dqiy_dx =

                -2*nix*niy;


        d2J_dqiy_dy =

                -2*pow(niy,2);


        d2J_dqiy_dz =

                -2*niy*niz;


        d2J_dqiy_da =

                -niy*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dqiy_db =

                -niy*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dqiy_dc =

                -niy*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));


        d2J_dqiz_dx =

                -2*nix*niz;


        d2J_dqiz_dy =

                -2*niy*niz;


        d2J_dqiz_dz =

                -2*pow(niz,2);


        d2J_dqiz_da =

                -niz*(2*niy*(piz*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) - piy*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c)) + pix*cos(a)*cos(b)) - 2*nix*(piy*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c)) - piz*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + pix*cos(b)*sin(a)));


        d2J_dqiz_db =

                -niz*(2*niy*(piz*cos(b)*cos(c)*sin(a) - pix*sin(a)*sin(b) + piy*cos(b)*sin(a)*sin(c)) - 2*niz*(pix*cos(b) + piz*cos(c)*sin(b) + piy*sin(b)*sin(c)) + 2*nix*(piz*cos(a)*cos(b)*cos(c) - pix*cos(a)*sin(b) + piy*cos(a)*cos(b)*sin(c)));


        d2J_dqiz_dc =

                -niz*(2*nix*(piy*(sin(a)*sin(c) + cos(a)*cos(c)*sin(b)) + piz*(cos(c)*sin(a) - cos(a)*sin(b)*sin(c))) - 2*niy*(piy*(cos(a)*sin(c) - cos(c)*sin(a)*sin(b)) + piz*(cos(a)*cos(c) + sin(a)*sin(b)*sin(c))) + 2*niz*(piy*cos(b)*cos(c) - piz*cos(b)*sin(c)));



        d2J_dZdX_temp <<    d2J_dpix_dx,    d2J_dpiy_dx,	d2J_dpiz_dx,  	   d2J_dqix_dx,    d2J_dqiy_dx,	   d2J_dqiz_dx,
                            d2J_dpix_dy,    d2J_dpiy_dy,	d2J_dpiz_dy,   	   d2J_dqix_dy,    d2J_dqiy_dy,	   d2J_dqiz_dy,
                            d2J_dpix_dz,    d2J_dpiy_dz,        d2J_dpiz_dz,       d2J_dqix_dz,    d2J_dqiy_dz,    d2J_dqiz_dz,
                            d2J_dpix_da,    d2J_dpiy_da,        d2J_dpiz_da,       d2J_dqix_da,    d2J_dqiy_da,    d2J_dqiz_da,
                            d2J_dpix_db,    d2J_dpiy_db,        d2J_dpiz_db,       d2J_dqix_db,    d2J_dqiy_db,    d2J_dqiz_db,
                            d2J_dpix_dc,    d2J_dpiy_dc,        d2J_dpiz_dc,       d2J_dqix_dc,    d2J_dqiy_dc,    d2J_dqiz_dc;


        d2J_dZdX.block<6,6>(0,6*k) = d2J_dZdX_temp;

    }


    //By reaching here both the matrices d2J_dX2 and d2J_dZdX are calculated and lets print those values out;

    //std::cout << "\n Finally here are the two matrices \n\n" << "d2J_dX2 = \n " << d2J_dX2 <<std::endl;
    //std::cout << "\n\n\n" << "d2J_dZdX = \n " << d2J_dZdX <<std::endl;




    /**************************************
     *
     * Here we create the matrix cov(z) as mentioned in Section 3.3 in the paper, "Covariance of ICP with 3D Point to Point and Point to Plane Error Metrics"
     *
     * ************************************/
    Eigen::MatrixXd cov_z(6*n,6*n);
    cov_z = 0.01 * Eigen::MatrixXd::Identity(6*n,6*n);



    ICP_COV =  d2J_dX2.inverse() * d2J_dZdX * cov_z * d2J_dZdX.transpose() * d2J_dX2.inverse();


    std::cout << "\n\n********************** \n\n" << "ICP_COV = \n" << ICP_COV <<"\n*******************\n\n"<< std::endl;

    std::cout << "\nSuccessfully Computed the ICP's Covariance !!!\n" << std::endl;

}


void cbshot::calculate_cov(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out, Eigen::Matrix4f & iG_transf,Eigen::MatrixXd & iG_cov)
{
    // Two Input Point Clouds
  /*   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
 */

    // /************************************/
    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data_bunny.pcd", *cloud_in) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //     return (-1);
    // }

    // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../model_bunny.pcd", *cloud_out) == -1) //* load the file
    // {
    //     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    //     return (-1);
    // }
    // /**************************************/


    //  std::cout << "Status : Loaded both the point clouds and started to compute SHOT descriptors" << endl;



    /***************************************************/
    // Model's Normals should be calculated..checked via (RPi-Qi + T).Ni
    // Here Ni is the Normal of Qi
    //hence we need to calculate the normal of model

    //    Pi is data -> cloud_in
    //    Qi is model -> cloud_out
    /***************************************************/




     /******************************************************
      *
      *
      *
      * Below, we bring the two input point clouds close to each other, as ICP can converge to a global minimum
      * We do it by matching keypoints by feature descriptors, then estimating the consensual transformation via RANSAC
      * Then transform the point clouds and pass them for ICP refinement
      *
      *
      *
      * ****************************************************/




    cbshot cb;

    cb.cloud1 = cloud_in;
    cb.cloud2 = cloud_out;

    cb.calculate_voxel_grid_keypoints(0.05);
    cb.calculate_normals(0.02);
    cb.calculate_SHOT(0.30);



    pcl::Correspondences corresp;
    pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> shot_corr;
    shot_corr.setInputSource(cb.cloud1_shot.makeShared());
    shot_corr.setInputTarget(cb.cloud2_shot.makeShared());
    std::cout << "Status : Calculated SHOT descriptors and finding the correspondences" << endl;
    cout << "May take a while...depends on the number of feature descriptors and its support size" << endl;
    shot_corr.determineReciprocalCorrespondences(corresp);


    pcl::CorrespondencesConstPtr correspond = boost::make_shared< pcl::Correspondences >(corresp);

    pcl::Correspondences corr;
    pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ > Ransac_based_Rejection;
    Ransac_based_Rejection.setInputSource(cb.cloud1_keypoints.makeShared());
    Ransac_based_Rejection.setInputTarget(cb.cloud2_keypoints.makeShared());
    Ransac_based_Rejection.setInputCorrespondences(correspond);
    Ransac_based_Rejection.getCorrespondences(corr);

    Eigen::Matrix4f ransac_mat = Ransac_based_Rejection.getBestTransformation();
    cout << "RANSAC based Transformation Matrix : \n" << ransac_mat << endl;

    pcl::transformPointCloud(cloud_in, cloud_in, ransac_mat);


    std::cout << "Now a window with the established correspondences between source and target keypoints pops up!" << endl;
    cout << "Based on these correspondences, the source and target point clouds are moved closer and then ICP works on them" << endl;
    cout << "If the established correspondences are not good, then the estimated transformation is not reliable!" << endl;
    cout << "It means that there is high probability that ICP may converge to a local minima !!!" << endl;



    /******************************************
     * If the ICP covariance values do not make sense
     * or
     * the model and data are not registered properly
     * then
     * look at the correspondences established by feature matching after RANSAC
     * It will show if the transformation can make two input point clouds close to each other
     *
     * Please remember that ICP works well for point clouds that are close enough
     * or else
     * ICP may converge to local minama..other way of saying it is that ICP fails
     * ***************************************/



    /************************************************************************************/

   // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("Feature Correspondences after RANSAC"));
   // viewer1->setBackgroundColor (0, 0, 0);

  //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cb.cloud1_keypoints.makeShared(), 255, 0, 0);
  //  viewer1->addPointCloud<pcl::PointXYZ> (cb.cloud1_keypoints.makeShared(), single_color3, "sample cloud3");
  //  viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud3");
  //  viewer1->initCameraParameters ();

    Eigen::Matrix4f mat1; mat1 << 1,0,0,3,0,1,0,0,0,0,1,0,0,0,0,1;
    pcl::PointCloud<pcl::PointXYZ> temp_here;
    pcl::transformPointCloud(cb.cloud2_keypoints,temp_here,mat1);

  //  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color4(temp_here.makeShared(), 0, 255, 0);
  //  viewer1->addPointCloud<pcl::PointXYZ> (temp_here.makeShared(), single_color4, "sample cloud4");
   // viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud4");

   // viewer1->addCorrespondences<pcl::PointXYZ>(cb.cloud1_keypoints.makeShared(), temp_here.makeShared(), corr, "correspondences");

    /**************************************************************************************/



    cout << "Started Point to Plane ICP" << endl;

    // Now do the Point to Plane ICP

    pcl::PointCloud<pcl::PointNormal> cld1, cld2;
    cbshot temp;
    temp.cloud1 = cloud_in;
    temp.cloud2 = cloud_out;
    temp.calculate_normals(0.02);
    pcl::copyPointCloud(temp.cloud1, cld1);// Take care that you pass the transformed clouds after the feature matching and RANSAC and not the input ones as they may not be close for global convergence
    pcl::copyPointCloud(temp.cloud2,cld2);
    pcl::copyPointCloud(temp.cloud1_normals, cld1);
    pcl::copyPointCloud(temp.cloud2_normals, cld2);

    Eigen::Matrix4f final_transformation;
    final_transformation << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;

    pcl::Correspondences correspondeces_reciprocal_shot;

    int iterations = 50;// We just hard code the number of ICP iterations to be 50
    for (int i = 0; i < iterations; i++)
    {

        pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal> corr_est;
        corr_est.setInputSource(cld1.makeShared()); // + setIndices(...)
        corr_est.setInputTarget(cld2.makeShared());
        corr_est.determineReciprocalCorrespondences(correspondeces_reciprocal_shot);
        //cout << "No. of Reciprocal Correspondences : " << correspondeces_reciprocal_shot.size() << endl;


        Eigen::Matrix4f transform_eigen;
        pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> pp_lls;
        pp_lls.estimateRigidTransformation (cld1, cld2, correspondeces_reciprocal_shot, transform_eigen);


        // rotate/transform data based on this estimated transformation
        pcl::transformPointCloud(cld1, cld1, transform_eigen);


        // accumulate incremental tf
        final_transformation = transform_eigen * final_transformation;
        iG_transf = final_transformation;

    }


    cout <<  "Final transformation (feature matching * ICP alone)  :\n" << ransac_mat * final_transformation << endl;

    cout << "*************\n!!! Here we estimate the ICP's covariance of transformation between closely moved point clouds !!! \n*************"<< endl;

    //cout << "No. of Reciprocal Correspondences : " << correspondeces_reciprocal_shot.size() << endl;


    std::vector<int> data_idx;
    std::vector<int> model_idx;

    pcl::Correspondence temp1;
    for (int i = 0; i < correspondeces_reciprocal_shot.size(); i++)
    {
        temp1 = correspondeces_reciprocal_shot[i];
        data_idx.push_back(temp1.index_query);
        model_idx.push_back(temp1.index_match);
    }


    pcl::PointCloud<pcl::PointNormal> data_pi; //Put all the pi in this cloud and its size will be equal to number of correspondences
    pcl::PointCloud<pcl::PointNormal> model_qi;// Put all the qi in this cloud and its size will be equal to number of correspondences

    pcl::copyPointCloud(cld1,data_idx,data_pi);
    pcl::copyPointCloud(cld2,model_idx,model_qi);

    Eigen::MatrixXd ICP_COV(6,6);
    ICP_COV = Eigen::MatrixXd::Zero(6,6);



    calculate_ICP_COV(data_pi, model_qi, final_transformation, ICP_COV);
    iG_cov = ICP_COV;

  //  std::cout << "\nComputational Time for ICP's covariance computation alone (in sec) = " ;

   // std::cout << "\n\n" << "ICP_COV Trace = \n\n" << ICP_COV.trace() <<"\n\n\n"<<   std::endl;






    /***************************************************/

   /*  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Registered Point Clouds after ICP"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color1(cld1.makeShared(), 255, 0, 0);
    viewer->addPointCloud<pcl::PointNormal> (cld1.makeShared(), single_color1, "sample cloud1");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud1");
    viewer->initCameraParameters ();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color2(cld2.makeShared(), 0, 255, 0);
    viewer->addPointCloud<pcl::PointNormal> (cld2.makeShared(), single_color2, "sample cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud2");

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    } */


    /***************************************************/

}
