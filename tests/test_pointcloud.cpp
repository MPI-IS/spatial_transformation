// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 * @brief Test functions from pointcloud.hpp
 * @copyright 2023 Max Planck Gesellschaft.  All rights reserved.
 */
#include <gtest/gtest.h>
#include <Eigen/Eigen>

#include <spatial_transformation/pointcloud.hpp>

#include "utils.hpp"

using spatial_transformation::compute_mean_transform_error;
using spatial_transformation::compute_transformation_between_point_clouds;
using spatial_transformation::json;
using spatial_transformation::json_point_cloud_to_eigen;
using spatial_transformation::read_point_clouds_from_json_file;

TEST(TestPointcloud, compute_mean_transform_error)
{
    Eigen::Matrix3Xd source, target1, target2;

    source.resize(Eigen::NoChange, 2);
    target1.resize(Eigen::NoChange, 2);
    target2.resize(Eigen::NoChange, 2);

    source.col(0) << 0, 0, 0;
    source.col(1) << 1, 1, 1;

    target1.col(0) << 0.1, -0.2, 0.3;
    target1.col(1) << 1.1, 0.8, 1.3;

    target2.col(0) << 0, 0, 0;
    target2.col(1) << 1, -1, 1;

    Eigen::Isometry3d tf_id, tf_translate, tf_rotate;

    tf_id.setIdentity();
    tf_translate.setIdentity();
    tf_translate.translation() << 0.1, -0.2, 0.3;

    tf_rotate.setIdentity();
    // rotate 90 deg. around x-axis
    tf_rotate.linear() << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0;

    // distance of translation (0.1, -0.2, 0.3) = 0.37416573867739417

    ASSERT_DOUBLE_EQ(compute_mean_transform_error(source, source, tf_id), 0.0);
    ASSERT_DOUBLE_EQ(
        compute_mean_transform_error(source, target1, tf_translate), 0.0);
    ASSERT_DOUBLE_EQ(compute_mean_transform_error(source, target2, tf_rotate),
                     0.0);
    ASSERT_DOUBLE_EQ(compute_mean_transform_error(source, source, tf_translate),
                     0.37416573867739417);
    ASSERT_DOUBLE_EQ(compute_mean_transform_error(source, target1, tf_id),
                     0.37416573867739417);
    ASSERT_DOUBLE_EQ(compute_mean_transform_error(source, target2, tf_id),
                     1.0);  // one point is same, other is 2.0 displaced
}

TEST(TestPointcloud, json_point_cloud_to_eigen)
{
    json data = json::parse(R"(
        [
            {"from": [0.0, 0.0, 0.0], "to": [1.0, 0.0, 0.0]},
            {"from": [1.0, 0.0, 0.0], "to": [2.0, 0.0, 0.0]},
            {"from": [1.1, 0.2, 0.3], "to": [2.5, 0.6, 0.7]},
            {"from": [0.0, 1.0, 1.0], "to": [1.0, -1.0, 1.0]}
        ]
)");

    Eigen::Matrix3Xd from, to;
    std::tie(from, to) = json_point_cloud_to_eigen(data, "from", "to");

    Eigen::Matrix3Xd expected_from, expected_to;
    expected_from.resize(Eigen::NoChange, 4);
    expected_to.resize(Eigen::NoChange, 4);
    expected_from.col(0) << 0.0, 0.0, 0.0;
    expected_from.col(1) << 1.0, 0.0, 0.0;
    expected_from.col(2) << 1.1, 0.2, 0.3;
    expected_from.col(3) << 0.0, 1.0, 1.0;
    expected_to.col(0) << 1.0, 0.0, 0.0;
    expected_to.col(1) << 2.0, 0.0, 0.0;
    expected_to.col(2) << 2.5, 0.6, 0.7;
    expected_to.col(3) << 1.0, -1.0, 1.0;

    ASSERT_MATRIX_ALMOST_EQUAL(from, expected_from);
    ASSERT_MATRIX_ALMOST_EQUAL(to, expected_to);
}

TEST(TestPointcloud, json_point_cloud_to_eigen_bad_input)
{
    json data_good = json::parse(R"(
        [
            {"from": [0.0, 0.0, 0.0], "to": [1.0, 0.0, 0.0]},
            {"from": [0.0, 1.0, 1.0], "to": [1.0, -1.0, 1.0]}
        ]
)");

    json data_bad1 = json::parse(R"(
        {"from": [0.0, 0.0, 0.0], "to": [1.0, 0.0, 0.0]}
)");

    json data_bad2 = json::parse(R"(
        [
            {"from": "bad", "to": [1.0, 0.0, 0.0]}
        ]
)");

    EXPECT_THROW({ json_point_cloud_to_eigen(data_bad1, "from", "to"); },
                 std::invalid_argument);

    EXPECT_THROW({ json_point_cloud_to_eigen(data_bad2, "from", "to"); },
                 json::type_error);

    EXPECT_THROW({ json_point_cloud_to_eigen(data_good, "bad", "to"); },
                 json::out_of_range);

    EXPECT_THROW({ json_point_cloud_to_eigen(data_good, "from", "bad"); },
                 json::out_of_range);
}

TEST(TestPointcloud, read_joint_clouds_from_json_file)
{
    Eigen::Matrix3Xd from, to;
    std::tie(from, to) = read_point_clouds_from_json_file(
        "tests/data/pointcloud.json", "from", "to");

    Eigen::Matrix3Xd expected_from, expected_to;
    expected_from.resize(Eigen::NoChange, 4);
    expected_to.resize(Eigen::NoChange, 4);
    expected_from.col(0) << 0.0, 0.0, 0.0;
    expected_from.col(1) << 1.0, 0.0, 0.0;
    expected_from.col(2) << 1.1, 0.2, 0.3;
    expected_from.col(3) << 0.0, 1.0, 1.0;
    expected_to.col(0) << 1.0, 0.0, 0.0;
    expected_to.col(1) << 2.0, 0.0, 0.0;
    expected_to.col(2) << 2.5, 0.6, 0.7;
    expected_to.col(3) << 1.0, -1.0, 1.0;

    ASSERT_MATRIX_ALMOST_EQUAL(from, expected_from);
    ASSERT_MATRIX_ALMOST_EQUAL(to, expected_to);
}

TEST(TestPointcloud, read_joint_clouds_from_json_file__file_not_found)
{
    EXPECT_THROW(
        {
            read_point_clouds_from_json_file(
                "tests/data/does_not_exist.json", "from", "to");
        },
        std::system_error);
}

TEST(TestPointcloud, compute_transformation_between_point_clouds)
{
    Eigen::Matrix3Xd from_points, to_points;
    from_points.resize(Eigen::NoChange, 4);
    to_points.resize(Eigen::NoChange, 4);
    from_points.col(0) << 0.0, 0.0, 0.0;
    from_points.col(1) << 1.0, 0.0, 0.0;
    from_points.col(2) << 1.1, 0.2, 0.3;
    from_points.col(3) << 0.0, 1.0, 1.0;
    to_points.col(0) << 1.0, 0.0, -0.3;
    to_points.col(1) << 2.0, 0.0, -0.3;
    to_points.col(2) << 2.1, 0.2, 0.0;
    to_points.col(3) << 1.0, 1.0, 0.7;

    Eigen::Isometry3d tf;
    double error;
    std::tie(tf, error) =
        compute_transformation_between_point_clouds(from_points, to_points);

    EXPECT_NEAR(error, 0.0, 0.0001);
    ASSERT_MATRIX_ALMOST_EQUAL(tf.translation(), Eigen::Vector3d(1.0, 0, -0.3));
    ASSERT_MATRIX_ALMOST_EQUAL(tf.rotation(), Eigen::Matrix3d::Identity());
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
