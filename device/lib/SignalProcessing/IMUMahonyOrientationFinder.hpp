#include <IMUOrientationFinder.hpp>
#include <Quaternion.hpp>

class IMUMahonyOrientationFinder : public IMUOrientationFinder<Quaternion> {
public:
  Quaternion find(IMUSample &sample) override {}

private:
};
