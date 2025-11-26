#pragma once

#include <string>
#include "LimelightHelpers.h"
#include <frc/Servo.h>

class LimelightMeta
{
private:
    struct Vector3
    {
        double x, y, z;
    };

    struct View
    {
        // meters
        double tx;
        double ty;
        double tz;

        // degrees
        double pitch;
        double yaw;
        double roll;

        View operator+(const View &other)
        {
            return View{other.tx + this->tx,
                        other.ty + this->ty,
                        other.tz + this->tz,
                        other.pitch + this->pitch,
                        other.yaw + this->yaw,
                        other.roll + this->roll};
        }
        View operator+(const Vector3 &other)
        {
            return View{other.x + this->tx,
                        other.y + this->ty,
                        other.z + this->tz,
                        this->pitch,
                        this->yaw,
                        this->roll};
        }

        View operator-(const View &other)
        {
            return View{this->tx - other.tx,
                        this->ty - other.ty,
                        this->tz - other.tz,
                        this->pitch - other.pitch,
                        this->yaw - other.yaw,
                        this->roll - other.roll};
        }
        void print(std::string pre_str = "")
        {
            printf("%s: %f, %f, %f; %f %f %f\n", pre_str.c_str(), tx, ty, tz, pitch, yaw, roll);
        }
    };

    static constexpr struct View view_limelight
    {
        - 0.4029, 0, 0.1651,
            0, 10, 180
    };

    static constexpr struct View id_2_apriltag
    {
        1.5113, 0, 0, 0, 0, 0
    };

    Vector3
    vector_rotate(double x, double y, double z, double roll, double pitch,
                  double yaw)
    {
        double tx = (std::cos(pitch) * std::cos(roll)) * x +
                    (std::sin(yaw) * std::sin(pitch) * std::cos(roll) -
                     std::cos(yaw) * std::sin(roll)) *
                        y +
                    (std::cos(yaw) * std::sin(pitch) * std::cos(roll) +
                     std::sin(yaw) * sin(roll)) *
                        z;
        double ty = (std::cos(pitch) * std::sin(roll)) * x +
                    (std::sin(yaw) * std::sin(pitch) * std::sin(roll) +
                     std::cos(yaw) * std::cos(roll)) *
                        y +
                    (std::cos(yaw) * sin(pitch) * std::sin(roll) -
                     std::sin(yaw) * std::cos(roll)) *
                        z;

        double tz = (-std::sin(pitch)) * x + (std::sin(yaw) * std::cos(pitch)) * y +
                    (std::cos(yaw) * std::cos(yaw)) * z;

        return {(tx), (ty), (tz)};
    }

    std::string primary_limelight_name;
    std::string secondary_limelight_name;
    frc::Servo gilesdar_servo{2};

    View process_tag(std::vector<double> &raw_tag)
    {
        double tx = raw_tag[0];
        double ty = raw_tag[1];
        double tz = raw_tag[2];
        double pitch = raw_tag[3];
        double yaw = raw_tag[4];
        double roll = raw_tag[5];

        return View{tx, ty, tz, pitch, yaw, roll};
    }

public:
    LimelightMeta(std::string primary_limelight_name, std::string secondary_limelight_name)
        : primary_limelight_name(primary_limelight_name), secondary_limelight_name(secondary_limelight_name)
    {
        wpi::PortForwarder::GetInstance().Add(5801, "172.29.0.1", 5801);
        wpi::PortForwarder::GetInstance().Add(5802, "172.29.0.1", 5802);
        wpi::PortForwarder::GetInstance().Add(5803, "172.29.0.1", 5803);
        wpi::PortForwarder::GetInstance().Add(5804, "172.29.0.1", 5804);
        wpi::PortForwarder::GetInstance().Add(5805, "172.29.0.1", 5805);
        wpi::PortForwarder::GetInstance().Add(5806, "172.29.0.1", 5806);
        wpi::PortForwarder::GetInstance().Add(5807, "172.29.0.1", 5807);
        wpi::PortForwarder::GetInstance().Add(5808, "172.29.0.1", 5808);
        wpi::PortForwarder::GetInstance().Add(5809, "172.29.0.1", 5809);
    }

    void get_position()
    {
        printf("%s\n", primary_limelight_name.c_str());
        auto primary_raw_tags = LimelightHelpers::getLimelightNTDoubleArray(primary_limelight_name, "targetpose_cameraspace");
        auto secondary_raw_tags = LimelightHelpers::getLimelightNTDoubleArray(secondary_limelight_name, "targetpose_cameraspace");

        auto primary_in_robot_space = LimelightHelpers::getLimelightNTDoubleArray(primary_limelight_name, "targetpose_robotspace");
        View tag_in_robot_space = process_tag(primary_in_robot_space);

        tag_in_robot_space.print("tag in to robot space");

        View tag_in_camera_space = process_tag(primary_raw_tags);
        tag_in_camera_space.print("tag in camera space");

        Vector3 rotted_point = vector_rotate(tag_in_camera_space.tx, tag_in_camera_space.ty, tag_in_camera_space.tz, -view_limelight.roll, -view_limelight.pitch, -view_limelight.yaw);
        tag_in_camera_space.tx = rotted_point.x;
        tag_in_camera_space.ty = rotted_point.y;
        tag_in_camera_space.tz = rotted_point.z;

        View tag_in_robot_space_calculated = tag_in_camera_space - view_limelight;
        tag_in_robot_space_calculated.print("tag in robot space calcuated");
    }

    ~LimelightMeta()
    {
    }
};
