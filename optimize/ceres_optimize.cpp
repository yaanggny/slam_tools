#include "ceres_optimize.h"
#include "feature_extract.h"
#include "transform.h"
#include "data_type.h"
#include "log_utils.h"
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/types.h>



struct PerpendCost
{
    PerpendCost(const MPoint3d& _oc, const MPoint3d& _cc1, const MPoint3d& _nl)
        : oc(_oc), cc1(_cc1), nl(_nl) {}
    
    template<typename T>
    bool operator()(const T* const rot, T* e) const
    {
        T v[3];
        T nl_[3] = {T(nl.x), T(nl.y), T(nl.z)};
        ceres::AngleAxisRotatePoint(rot, nl_, v);
        MPoint3d vc = oc - cc1;
        e[0] = vc.x*v[0] + vc.y*v[1] + vc.z*v[2];
        return true;
    }
    MPoint3d oc;
    MPoint3d cc1;
    MPoint3d nl;
};

struct AlignCost
{
    AlignCost(const MPoint3d& _nc, const MPoint3d& _nl)
        :nc(_nc), nl(_nl) {}
    
    template<typename T>
    bool operator()(const T* const rot, T* e) const
    {
        T v[3];
        T nl_[3] = {T(nl.x), T(nl.y), T(nl.z)};
        ceres::AngleAxisRotatePoint(rot, nl_, v);
        T t[3] = {v[0] - nc.x, v[1] - nc.y, v[2]-nc.z};
        e[0] = sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        return true;
    }
    MPoint3d nc, nl;    
};

struct PointDistCost
{
    PointDistCost(const MPoint3d& _oc, const MPoint3d& _ol)
        : oc(_oc), ol(_ol) {}
    
    template<typename T>
    bool operator()(const T* const rot, const T* const vt, T* e) const
    {
        T v[3];
        T ol_[3] = {T(ol.x), T(ol.y), T(ol.z)};
        ceres::AngleAxisRotatePoint(rot, ol_, v);
        v[0] += vt[0];
        v[1] += vt[1];
        v[2] += vt[2];
        T t[3] = {v[0] - oc.x, v[1] - oc.y, v[2]-oc.z};
        e[0] = sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        return true;
    }
    MPoint3d oc, ol;    
};



static std::vector<double> rotm2eul(const cv::Mat& mat)
{
    std::vector<double> euler(3);
    euler[0] = atan2(mat.at<double>(2, 1), mat.at<double>(2, 2));  // rotation about x axis: roll
    euler[1] = atan2(-mat.at<double>(2, 0),
                        sqrt(mat.at<double>(2, 1) * mat.at<double>(2, 1) + mat.at<double>(2, 2) * mat.at<double>(2, 2)));
    euler[2] = atan2(mat.at<double>(1, 0), mat.at<double>(0, 0));  // rotation about z axis: yaw
    return euler;
}

void optimize(const std::vector<PairFeature>& fts, Eigen::Vector3d& euler_angles, Eigen::Vector3d& trans_vec)
{
    bool useCorner = true;
    int nk = useCorner? 4: 1;
    int ns = fts.size();
    cv::Mat camCenters = cv::Mat(ns*nk, 3, CV_64FC1);
    cv::Mat camNormals = cv::Mat(ns, 3, CV_64FC1);
    cv::Mat lidarCenters = cv::Mat(ns*nk, 3, CV_64FC1);
    cv::Mat lidarNormals = cv::Mat(ns, 3, CV_64FC1);   

    int row = 0;
    int rc = 0;
    for(auto& s: fts)
    {
        cvImageT cn = cvImageT(s.camBNormal).reshape(1).t();
        cn.copyTo(camNormals.row(row));
        cvImageT cc = cvImageT(s.camBCenter).reshape(1).t();
        cc.copyTo(camCenters.row(rc));
        cvImageT ln = cvImageT(s.lidarBNormal).reshape(1).t();
        ln.copyTo(lidarNormals.row(row));
        cvImageT lc = cvImageT(s.lidarBCenter).reshape(1).t();
        lc.copyTo(lidarCenters.row(rc));
        if(useCorner)
        {
            for(int i = 0; i < 3; i++)
            {
                cvImageT cc = cvImageT(s.camCorners[i]).reshape(1).t();
                cc.copyTo(camCenters.row(rc));
                cvImageT lc = cvImageT(s.lidarCorners[i]).reshape(1).t();
                lc.copyTo(lidarCenters.row(rc));
                rc++;
            }
        }
        row++;
        rc++;        
    }

    cvImageT NN = lidarNormals.t() * lidarNormals;  // matrix is row-wise
    cvImageT NM = lidarNormals.t() * camNormals;
    cvImageT UNR = (NN.inv() * NM).t();
    std::cout << "UNR: " << UNR << std::endl;
    std::cout << "UNR*UNR.t: " << cvImageT(UNR*UNR.t()).reshape(1, 1) << std::endl;

    std::vector<double> euler = rotm2eul(UNR);
    cvImageT UNR_t = UNR.t();
    std::cout << UNR_t << std::endl;
    double rot0[9];
    double rot_angle_axis[3];
    memcpy(rot0, UNR_t.data, 9*sizeof(double));
    ceres::RotationMatrixToAngleAxis(rot0, rot_angle_axis);
    
    using T = double;
    for(const auto& ft: fts)
    {
        T v[3];
        const MPoint3d& nl = ft.lidarBNormal;
        T nl_[3] = {T(nl.x), T(nl.y), T(nl.z)};
        ceres::AngleAxisRotatePoint(rot_angle_axis, nl_, v);
        const MPoint3d& oc = ft.camBCenter, &cc1 =ft.camCorners[0];
        MPoint3d vc = oc - cc1;
        double e1 = vc.x*v[0] + vc.y*v[1] + vc.z*v[2];

        const MPoint3d& nc = ft.camBNormal;
        T t[3] = {v[0] - nc.x, v[1] - nc.y, v[2]-nc.z};
        double e2 = sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        printf("id= %d   e= %.3f   e2=%.3f\n", ft.sampleId, e1, e2);
    }    

    // return;

    ceres::Problem problem;
    for(const auto& ft: fts)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PerpendCost, 1, 3>
            (new PerpendCost(ft.camBCenter, ft.camCorners[0], ft.lidarBNormal)), nullptr, rot_angle_axis);
        if(useCorner)
        {
            for(int i = 0; i < 3; i++)
            {
                problem.AddResidualBlock(new ceres::AutoDiffCostFunction<PerpendCost, 1, 3>
                    (new PerpendCost(ft.camCorners[i], ft.camCorners[3], ft.lidarBNormal)), nullptr, rot_angle_axis);
            }
        }
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<AlignCost, 1, 3>
            (new AlignCost(ft.camBNormal, ft.lidarBNormal)), nullptr, rot_angle_axis);
    }

    ceres::Solver::Options opts;
    opts.max_num_iterations = 200;
    // opts.linear_solver_type = ceres::DENSE_QR;
    // opts.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    opts.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    opts.function_tolerance = 1e-8;
    opts.minimizer_progress_to_stdout = true;

    mio_LOGI("init: r,p,y: %.3f, %.3f, %.3f", euler[0], euler[1], euler[2]);
    mio_LOGI("init: angle axis: %.3f, %.3f, %.3f", rot_angle_axis[0], rot_angle_axis[1], rot_angle_axis[2]);

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    double rot1[9];
    double rot[9];
    ceres::AngleAxisToRotationMatrix(rot_angle_axis, rot1);
    cvImageT ro_t(3, 3, CV_64FC1, rot1);
    euler = rotm2eul(cvImageT(3, 3, CV_64FC1, rot1));
    std::cout << "rotMat(C2L): " << ro_t << std::endl;
    mio_LOGI("final: r,p,y(C2L): %.3f, %.3f, %.3f", euler[0], euler[1], euler[2]);
    mio_LOGI("final: angle axis(C2L): %.3f, %.3f, %.3f", rot_angle_axis[0], rot_angle_axis[1], rot_angle_axis[2]);

    // transpose to row-wise (ceres to openCV)
    cvImageT ro = ro_t.t();
    memcpy(rot, ro.data, 9*sizeof(double));
    euler = rotm2eul(ro);
    mio_LOGI("final: r,p,y(L2C): %.3f, %.3f, %.3f", euler[0], euler[1], euler[2]);
    std::cout << "rotMat(L2C): " << ro << std::endl;
    cvImageT cp_trans = ro * lidarCenters.t();
    cvImageT trans_diff = camCenters.t() - cp_trans;
    cvImageT sum_diff;
    cv::reduce(trans_diff, sum_diff, 1, cv::REDUCE_SUM, CV_64FC1);  // CV_REDUCE_SUM
    sum_diff /= trans_diff.cols;

    double translation[3] = {sum_diff.at<double>(0), sum_diff.at<double>(1), sum_diff.at<double>(2)};
    mio_LOGI("final: translation x,y,z(L2C): %.3f, %.3f, %.3f", translation[0], translation[1], translation[2]);
    for(int i = 0; i < 3; i++)
    {
        euler_angles[i] = euler[2-i];
        trans_vec[i] = translation[i];
    }

    Eigen::Affine3d mat = Eigen::Affine3d::Identity();
    mat.linear() = EulerAnglesToRotationMatrix(Eigen::Vector3d(euler[2], euler[1], euler[0]));
    mat.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    Eigen::Affine3d ri = mat.inverse(Eigen::TransformTraits::Affine);

    std::cout << "lidar2cam: " << mat.matrix() << std::endl;
    std::cout << "cam2lidar: " << ri.matrix() << std::endl;

    ceres::Problem problem2;
    for(const auto& ft: fts)
    {
        problem2.AddResidualBlock(new ceres::AutoDiffCostFunction<PerpendCost, 1, 3>
            (new PerpendCost(ft.camBCenter, ft.camCorners[0], ft.lidarBNormal)), nullptr, rot_angle_axis);

        problem2.AddResidualBlock(new ceres::AutoDiffCostFunction<PointDistCost, 1, 3, 3>
            (new PointDistCost(ft.camBCenter, ft.lidarBCenter)), nullptr, rot_angle_axis, translation);
        if(useCorner)
        {
            for(int i = 0; i < 3; i++)
            {
                problem2.AddResidualBlock(new ceres::AutoDiffCostFunction<PerpendCost, 1, 3>
                    (new PerpendCost(ft.camCorners[i], ft.camCorners[3], ft.lidarBNormal)), nullptr, rot_angle_axis);
                problem2.AddResidualBlock(new ceres::AutoDiffCostFunction<PointDistCost, 1, 3, 3>
                    (new PointDistCost(ft.camCorners[i], ft.lidarCorners[i])), nullptr, rot_angle_axis, translation);
            }
        }
        problem2.AddResidualBlock(new ceres::AutoDiffCostFunction<AlignCost, 1, 3>
            (new AlignCost(ft.camBNormal, ft.lidarBNormal)), nullptr, rot_angle_axis);
    }

    ceres::Solve(opts, &problem2, &summary);
    std::cout << summary.BriefReport() << "\n";

    ceres::AngleAxisToRotationMatrix(rot_angle_axis, rot1);
    ro_t = cvImageT(3, 3, CV_64FC1, rot1);
    euler = rotm2eul(cvImageT(3, 3, CV_64FC1, rot1));
    std::cout << "rotMat(C2L): " << ro_t << std::endl;
    mio_LOGI("final: r,p,y(C2L): %.3f, %.3f, %.3f", euler[0], euler[1], euler[2]);

    ro = ro_t.t();
    euler = rotm2eul(ro);
    mio_LOGI("final: r,p,y(L2C): %.3f, %.3f, %.3f", euler[0], euler[1], euler[2]);
    std::cout << "rotMat(L2C): " << ro << std::endl;

    mat.linear() = EulerAnglesToRotationMatrix(Eigen::Vector3d(euler[2], euler[1], euler[0]));
    mat.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
    ri = mat.inverse(Eigen::TransformTraits::Affine);

    std::cout << "lidar2cam: " << mat.matrix() << std::endl;
    std::cout << "cam2lidar: " << ri.matrix() << std::endl;

    for(int i = 0; i < 3; i++)
    {
        euler_angles[i] = euler[2-i];
        trans_vec[i] = translation[i];
    }
}

void transform_l2c(const Eigen::Vector3d& rot_vec, const Eigen::Vector3d& trans_vec, Eigen::Matrix4d& transMat)
{
    Eigen::Affine3d mat = Eigen::Affine3d::Identity();
    mat.linear() = EulerAnglesToRotationMatrix(rot_vec);
    mat.translation() = trans_vec;
    transMat = mat.matrix();
}