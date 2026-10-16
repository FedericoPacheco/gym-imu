#pragma once
#include <Constants.hpp>
#include <IMUGravityRemover.hpp>
#include <Quaternion.hpp>

class IMUQuaternionGravityRemover : public IMUGravityRemover<Quaternion> {
public:
  void remove(IMUSample &sample, Quaternion &orientation) override {
    auto gB = orientation.conjugate().rotate(G_W);
    sample.a.x += gB[0];
    sample.a.y += gB[1];
    sample.a.z += gB[2];
  }
};