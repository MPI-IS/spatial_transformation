// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 * @brief Utility functions for computing transform between point clouds.
 * @copyright 2023 Max Planck Gesellschaft.  All rights reserved.
 *
 * The functions of this library can be used to compute the transformation
 * between two reference frames, given the positions of a point cloud in both
 * frames.
 *
 * Example:
 *
 * JSON file ``points.json``:
 *
 * @code{.json}
 *   [
 *     {"foo": [-0.79, 2.11, 0.27], "bar": [-0.54, -0.87, -0.62]},
 *     {"foo": [-0.93, 2.36, 0.26], "bar": [-0.68, -0.62, -0.63]},
 *     ...
 *   ]
 * @endcode
 *
 * Code to load the file and compute the transformation:
 *
 * @code{.cpp}
 *   // load point clouds from file
 *   const auto points = read_point_clouds_from_json_file(
 *           "points.json", "foo", "bar");
 *
 *   // compute transformation
 *   const auto &[transformation, mean_error] =
 *       compute_transformation_between_point_clouds(
 *           points.first, points.second);
 * @endcode
 */
#pragma once

#include <filesystem>

#include <spdlog/logger.h>
#include <Eigen/Eigen>

#include "thirdparty/json.hpp"

namespace spatial_transformation
{
using nlohmann::json;

/**
 * @brief Compute mean position error of point cloud transformation.
 *
 * Transform the source points using the given transformation and compute the
 * mean error to the expected target points.
 *
 * @param source_points  Point cloud that is to be transformed.
 * @param expected_target_points  Expected point positions after transformation.
 * @param transform  Transformation that is applied on source_points.
 *
 * @return Mean absolute position error of the transformed source points to
 *      expected_target_points.
 */
double compute_mean_transform_error(Eigen::Matrix3Xd source_points,
                                    Eigen::Matrix3Xd expected_target_points,
                                    Eigen::Isometry3d transform);

/**
 * @brief Extract 3d point cloud in two reference frames from a JSON data
 * structure.
 *
 * The JSON data is expected to be structured as a sequence of objects where
 * each object contains two members with names specified by first_key and
 * second_key.  They are expected to be lists of three numbers each,
 * representing the position of the same point in the first and the second
 * frame.
 *
 * Example (where "foo" and "bar" are first and second key):
 *
 * @code{.json}
 *    [
 *      {"foo": [-0.79, 2.11, 0.27], "bar": [-0.54, -0.87, -0.62]},
 *      {"foo": [-0.93, 2.36, 0.26], "bar": [-0.68, -0.62, -0.63]},
 *      ...
 *    ]
 * @endcode
 *
 * The positions are extracted and stored in a pair of 3xN matrices.
 *
 * @param json_data JSON structure as described above.
 * @param first_key Name of the entries containing points in the first frame.
 * @param second_key Name of the entries containing points in the second frame.
 *
 * @return Pair of 3xN matrices.  The first element contains all the points
 *      corresponding the first_key, the second element all the points
 *      corresponding to second_key.
 */
std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> json_point_cloud_to_eigen(
    const json &json_data,
    const std::string &first_key,
    const std::string &second_key);

/**
 * @brief Read 3d point cloud in two reference frames from a JSON file.
 *
 * Regarding the expected structure of the JSON file and the meaning of first
 * and second key, see @ref json_point_cloud_to_eigen.
 *
 * @param json_file  Path to the JSON file.
 * @param first_key Name of the entries containing points in the first frame.
 * @param second_key Name of the entries containing points in the second frame.
 *
 * @return Pair of 3xN matrices.  The first element contains all the points
 *      corresponding the first_key, the second element all the points
 *      corresponding to second_key.
 *
 * @throw std::system_error  If opening the file fails.
 * @throw std::runtime_error  If the file is not valid JSON or does not have the
 *      expected structure.
 */
std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> read_point_clouds_from_json_file(
    const std::filesystem::path &json_file,
    const std::string &first_key,
    const std::string &second_key);

/**
 * @brief Compute transformation between two point clouds.
 *
 * Expects as input two point clouds and computes a transformation that maps the
 * first one to the second.  This can be used to find the transformation between
 * two reference frames if a set of points is given in both frames.
 *
 * The two point clouds are expected to be of equal size and need to be ordered,
 * i.e. point ``from_points[i]`` corresponds to ``to_points[i]`` for all ``i``.
 *
 * @param from_points  Points in reference frame A.
 * @param to_points  Same points in reference frame B.
 *
 * @return Tuple ``(tf, mean_error)`` with
 *      - ``tf``: the transformation from A to B and
 *      - ``mean_error``: the mean absolute error of the transformation based on
 *         the given point clouds.
 */
std::tuple<Eigen::Isometry3d, double>
compute_transformation_between_point_clouds(const Eigen::Matrix3Xd &from_points,
                                            const Eigen::Matrix3Xd &to_points);

}  // namespace spatial_transformation
