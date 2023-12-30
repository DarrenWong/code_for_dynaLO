// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"
#include "tic_toc.h"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution)
{
    // init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    plannar_feature_with_weight = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    // downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    // kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();
    optimization_count = 2;
    use_reweight = lidar_param.use_reweight;
    use_remove_only = lidar_param.use_remove_only;

    std::cout << "use reweight:" << use_reweight << std::endl;
    std::cout << "use_remove_only:" << use_remove_only << std::endl;
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in)
{
    *laserCloudCornerMap += *edge_in;
    *laserCloudSurfMap += *surf_in;
    optimization_count = 12;
}

void OdomEstimationClass::updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_in, ros::Time &pointcloud_time)
{

    if (optimization_count > 2)
        optimization_count--;
	TicToc t_updatePointsToMap;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in, downsampledEdgeCloud, surf_in, downsampledSurfCloud);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(edge_in);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(surf_in);
    // downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    std::vector<int> surf_ids;
    std::vector<ceres::ResidualBlockId> planarResIDs;

    // ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if (laserCloudCornerMap->points.size() > 10 && laserCloudSurfMap->points.size() > 50)
    {
        kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
        kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

        double para_surf_weight[downsampledSurfCloud->points.size()];

        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
            ceres::LossFunction *loss_function = NULL;

            ceres::Problem::Options problem_options;
            ceres::Problem problem(problem_options);
            planarResIDs.clear();
            surf_ids.clear();
            planar_cost_function_id_.clear();
            dynamic_cost_function_id_.clear();
            static_cost_function_id_.clear();
            plannar_feature_with_weight->clear();
            if (iterCount == 0)
            {
                for (uint32_t j = 0; j < downsampledSurfCloud->points.size(); ++j)
                {
                    para_surf_weight[j] = 1;
                }
            }
            //  /* add parameter block for ambiguity parameter block */
            for (uint32_t i = 0; i < downsampledSurfCloud->points.size(); ++i)
            {

                problem.AddParameterBlock(para_surf_weight + i, 1);

                problem.SetParameterUpperBound(para_surf_weight + i, 0, 1.0);
                problem.SetParameterLowerBound(para_surf_weight + i, 0, 0.0);
            }

            problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
            // addEdgeCostFactor(downsampledEdgeCloud,laserCloudCornerMap,problem,loss_function);
            addSurfCostFactor(downsampledSurfCloud, laserCloudSurfMap, problem, loss_function, para_surf_weight, surf_ids, planarResIDs);

            ceres::Solver::Options options;
 
            options.max_num_iterations = 256;
            options.num_threads = 24;
            options.linear_solver_type = ceres::DENSE_SCHUR;
            options.minimizer_progress_to_stdout = false;

            ceres::Solver::Summary summary;

	        TicToc t_ceres_solver;
            ceres::Solve(options, &problem, &summary);
            // surf_after
            ceres::Problem::EvaluateOptions EvalOpts_surf;
            EvalOpts_surf.num_threads = 32;
            EvalOpts_surf.apply_loss_function = false;
            EvalOpts_surf.residual_blocks = planarResIDs;
            std::vector<double> residuals_surf;
            problem.Evaluate(EvalOpts_surf, nullptr, &residuals_surf, nullptr, nullptr);
            double dynamic_cost = 0;

            for (uint32_t j = 0; j < surf_ids.size(); ++j)
            {

                //LOG(INFO) << std::setprecision(15) << "timestamp: " << pointcloud_time.toSec() << " iteration:" << iterCount << " " << j << " out of " << surf_ids.size() - 1 << " " << para_surf_weight[surf_ids[j]] << " " << residuals_raw_surf[j] << " " << residuals_surf[j] << std::endl;
                dynamic_cost+= 0.5* std::pow(residuals_surf[j],2);
                plannar_feature_with_weight->points[j].intensity = para_surf_weight[surf_ids[j]];
            }
        }
    }
    else
    {
        printf("not enough points in map to associate, map error\n");
    }
    odom = Eigen::Isometry3d::Identity();
    odom.linear() = q_w_curr.toRotationMatrix();
    odom.translation() = t_w_curr;
    addPointsToMap(downsampledEdgeCloud, downsampledSurfCloud);
    printf("mapping optimization time %f %f \n", t_updatePointsToMap.toc(),pointcloud_time.toSec());

}

void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    po->intensity = pi->intensity;
    // po->intensity = 1.0;
}

void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &edge_pc_out, const pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &surf_pc_out)
{
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
}

void OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function)
{
    int corner_num = 0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(map_in->points[pointSearchInd[j]].x,
                                    map_in->points[pointSearchInd[j]].y,
                                    map_in->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;
            }
        }
    }
    if (corner_num < 20)
    {
        printf("not enough correct points");
    }
}

void OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, const pcl::PointCloud<pcl::PointXYZI>::Ptr &map_in, ceres::Problem &problem, ceres::LossFunction *loss_function, double *para_weight, std::vector<int> &surf_ids, std::vector<ceres::ResidualBlockId> &psrIDs)
{
    int surf_num = 0;
    int dynamic_num = 0;

    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {

            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = map_in->points[pointSearchInd[j]].x;
                matA0(j, 1) = map_in->points[pointSearchInd[j]].y;
                matA0(j, 2) = map_in->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * map_in->points[pointSearchInd[j]].x +
                         norm(1) * map_in->points[pointSearchInd[j]].y +
                         norm(2) * map_in->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {

                if (!use_reweight)
                {
                    //std::cout<<"do not use reweight"<<std::endl;
                    ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                    auto ID = problem.AddResidualBlock(cost_function, loss_function, parameters);
                    planar_cost_function_id_.emplace_back(cost_function);

                    // record the dynamic points to compute eigen value
                    if (pc_in->points[i].intensity < 1)
                    {
                        static_cost_function_id_.emplace_back(cost_function);
                    }
                    else
                    {
                        dynamic_cost_function_id_.emplace_back(cost_function);
                    }
                }
                else
                {
                    if (pc_in->points[i].intensity < 1)
                    {
                        // std::cout<<"use static point to join:"<<surf_num<<std::endl;

                        ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);
                        auto ID = problem.AddResidualBlock(cost_function, loss_function, parameters);
                        planar_cost_function_id_.emplace_back(cost_function);

                        surf_num++;
                    }
                    else if (!use_remove_only) // if use remove only, the dynamic point will not join to optimization
                    {
                        // std::cout<<"use dynamic point to joint:"<<dynamic_num<<std::endl;

                        dynamic_num++;
                        //add weighting term
                        ceres::CostFunction *cost_function_dynamic = new DynamicSurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);

                        auto ID = problem.AddResidualBlock(cost_function_dynamic, loss_function, parameters, para_weight + i);
                        surf_ids.emplace_back(i);
                        psrIDs.emplace_back(ID);
                        dynamic_cost_function_id_.emplace_back(cost_function_dynamic);
                        plannar_feature_with_weight->push_back(pc_in->points[i]);
                        //add penalty term
                        ceres::CostFunction *cost_function2 = new PenaltyCostFunction(i);
                        problem.AddResidualBlock(cost_function2, nullptr, para_weight + i);
                    }
                }
            }
        }
    }
    if (surf_num < 20)
    {
        printf("not enough correct points");
    }
}

void OdomEstimationClass::addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud)
{

    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledEdgeCloud->points[i], &point_temp);
        laserCloudCornerMap->push_back(point_temp);
    }

    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        pointAssociateToMap(&downsampledSurfCloud->points[i], &point_temp);
        laserCloudSurfMap->push_back(point_temp);
    }

    double x_min = +odom.translation().x() - 100;
    double y_min = +odom.translation().y() - 100;
    double z_min = +odom.translation().z() - 100;
    double x_max = +odom.translation().x() + 100;
    double y_max = +odom.translation().y() + 100;
    double z_max = +odom.translation().z() + 100;

    // ROS_INFO("size : %f,%f,%f,%f,%f,%f", x_min, y_min, z_min,x_max, y_max, z_max);
    cropBoxFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    cropBoxFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    cropBoxFilter.setNegative(false);

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpCorner(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpSurf(new pcl::PointCloud<pcl::PointXYZI>());
    cropBoxFilter.setInputCloud(laserCloudSurfMap);
    cropBoxFilter.filter(*tmpSurf);
    cropBoxFilter.setInputCloud(laserCloudCornerMap);
    cropBoxFilter.filter(*tmpCorner);

    downSizeFilterSurf.setInputCloud(tmpSurf);
    downSizeFilterSurf.filter(*laserCloudSurfMap);
    downSizeFilterEdge.setInputCloud(tmpCorner);
    downSizeFilterEdge.filter(*laserCloudCornerMap);
}

void OdomEstimationClass::getMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &laserCloudMap)
{

    *laserCloudMap += *laserCloudSurfMap;
    *laserCloudMap += *laserCloudCornerMap;
}

OdomEstimationClass::OdomEstimationClass()
{
}
