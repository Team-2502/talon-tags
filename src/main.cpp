#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <apriltag/frc/apriltag/AprilTagDetector.h>
#include <apriltag/frc/apriltag/AprilTagPoseEstimator.h>
#include <apriltag/frc/apriltag/AprilTagFieldLayout.h>
#include "data.h"

int main() {
    auto aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    frc::AprilTagDetector detector;

    detector.SetConfig(frc::AprilTagDetector::Config(12));

    detector.AddFamily("tag36h11", 0);

    frc::AprilTagPoseEstimator::Config poseEstConfig = {
            .tagSize = units::length::inch_t(6.5),
            .fx = 421.15,
            .fy = 421.15,
            .cx = 333.17,
            .cy = 245.52
    };

    frc::AprilTagPoseEstimator estimator = frc::AprilTagPoseEstimator(poseEstConfig);

    cv::VideoCapture cap("/dev/video0");

    cv::Mat mat;
    cv::Mat grayMat;

    cv::Scalar outlineColor = cv::Scalar(0, 255, 0);
    cv::Scalar crossColor = cv::Scalar(0, 0, 255);

    cap.set(cv::CAP_PROP_FRAME_WIDTH,640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,480);

    while (true) {
        auto start = std::chrono::steady_clock::now();

        if (!cap.read(mat))
            break;

        cv::rotate(mat, mat, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imwrite("out.png", mat);

        cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

        cv::Size g_size = grayMat.size();
        frc::AprilTagDetector::Results detections =
                detector.Detect(g_size.width, g_size.height, grayMat.data);

        for (const frc::AprilTagDetection *detection: detections) {
            frc::Transform3d pose = estimator.Estimate(*detection);

            if (detection->GetId() == 4) {
                auto distance = sqrt(pow(pose.X().value(), 2) + pow(pose.Y().value(), 2));
                std::string rot = units::angle::to_string(pose.Rotation().Angle().convert<units::degrees>());
                rot.pop_back();
                rot.pop_back();
                rot.pop_back();

                std::string distanceRotJson = "{ \"d\": " + std::to_string(distance) + ", " + "\"theta\": " + rot + "}";

                sendData("distance_angle", distanceRotJson);

                std::cout << distanceRotJson << std::endl;
            }

            // Z, Y
            frc::Pose3d tmpPose = frc::Pose3d(pose.Z(), pose.X(), pose.Y(), pose.Rotation());

            auto apriltagPose = aprilTagFieldLayout.GetTagPose(detection->GetId()).value();
            //auto tagPose = frc::Pose3d(apriltagPose.X(), apriltagPose.Y(), apriltagPose.Rotation().Angle());

            frc::Pose3d robotPose = tmpPose.RelativeTo(apriltagPose).TransformBy(frc::Transform3d(units::meter_t(0), units::meter_t(0), units::meter_t(0), frc::Rotation3d()));

            //std::cout << detection->GetId() << std::endl;

            /*std::cout << units::length::to_string(robotPose.X()) << ", "
            << units::length::to_string(robotPose.Y()) << ", " <<
            units::angle::to_string(robotPose.Rotation().Degrees())
            << std::endl;*/

            std::string xstr = units::length::to_string(robotPose.X());
            xstr.pop_back();
            xstr.pop_back();

            std::string ystr = units::length::to_string(robotPose.Y());
            ystr.pop_back();
            ystr.pop_back();

            std::string rstr = units::angle::to_string(robotPose.Rotation().Angle());
            rstr.pop_back();
            rstr.pop_back();
            rstr.pop_back();

            std::string jsonString = "{ \"x\": " + xstr + ", " + "\"y\": " + ystr + ", " + "\"theta\": " + rstr + "}";

            //std::cout << xstr << ", " << ystr << ", " << rstr << std::endl;
            //std::cout << jsonString << std::endl;
            
            frc::Rotation3d rotation = pose.Rotation();

            /*std::cout << "Translation: " + units::length::to_string(pose.X())
                         << ", " << units::length::to_string(pose.Y()) << ", "
                         << units::length::to_string(pose.Z()) << std::endl;

            std::cout << "Rotation: "
                         << units::angle::to_string(rotation.X()) << ", "
                         << units::angle::to_string(rotation.Y()) << ", "
                         << units::angle::to_string(rotation.Z()) << std::endl;*/

            sendData("set_position", jsonString);
        }

        auto end = std::chrono::steady_clock::now();

        std::cout << std::chrono::duration<double, std::milli>(end - start).count() << " ms" << std::endl;
        std::cout << "\x1B[2J\x1B[H";
    }

    return 0;
}