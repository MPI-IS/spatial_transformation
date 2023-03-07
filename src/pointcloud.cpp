// SPDX-License-Identifier: BSD-3-Clause

#include <spatial_transformation/pointcloud.hpp>

#include <fstream>

#include <fmt/core.h>
#include <fmt/ostream.h>

namespace spatial_transformation
{
double compute_mean_transform_error(Eigen::Matrix3Xd source_points,
                                    Eigen::Matrix3Xd expected_target_points,
                                    Eigen::Isometry3d transform)
{
    Eigen::Matrix3Xd transformed_points =
        transform * source_points.colwise().homogeneous();
    Eigen::Matrix3Xd diff = transformed_points - expected_target_points;
    Eigen::VectorXd norms = diff.colwise().norm();

    return norms.mean();
}

std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> json_point_cloud_to_eigen(
    const json &json_data,
    const std::string &first_key,
    const std::string &second_key)
{
    if (!json_data.is_array())
    {
        throw std::invalid_argument(
            "Invalid data structure.  Expected sequence.");
    }

    // load positions from trajectory into Eigen matrices, one column per point
    Eigen::Matrix3Xd first_points, second_points;
    const size_t n_points = json_data.size();
    first_points.resize(Eigen::NoChange, n_points);
    second_points.resize(Eigen::NoChange, n_points);
    for (size_t i = 0; i < n_points; i++)
    {
        std::array<double, 3> first_point, second_point;
        json_data[i].at(first_key).get_to(first_point);
        json_data[i].at(second_key).get_to(second_point);

        for (size_t j = 0; j < 3; j++)
        {
            first_points(j, i) = first_point[j];
            second_points(j, i) = second_point[j];
        }
    }

    return std::make_pair(first_points, second_points);
}

std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> read_point_clouds_from_json_file(
    const std::filesystem::path &json_file,
    const std::string &first_key,
    const std::string &second_key)
{
    std::ifstream in_stream(json_file);
    if (in_stream.fail())
    {
        throw std::system_error(
            errno,
            std::system_category(),
            fmt::format("Failed to open file '{}'", json_file.string()));
    }

    json trajectory;
    try
    {
        in_stream >> trajectory;
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error(
            fmt::format("Failed to parse JSON file '{}': {}",
                        json_file.string(),
                        e.what()));
    }

    try
    {
        return json_point_cloud_to_eigen(trajectory, first_key, second_key);
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error(
            fmt::format("Failed to read point clouds from JSON: {}", e.what()));
    }
}

std::tuple<Eigen::Isometry3d, double>
compute_transformation_between_point_clouds(const Eigen::Matrix3Xd &from_points,
                                            const Eigen::Matrix3Xd &to_points)
{
    // compute transformation using Umeyama algorithm
    Eigen::Isometry3d tf;
    tf.matrix() = Eigen::umeyama(from_points, to_points, false);

    double mean_error =
        compute_mean_transform_error(from_points, to_points, tf);

    return {tf, mean_error};
}

}  // namespace spatial_transformation
