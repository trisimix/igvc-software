
#ifndef SRC_FIELD_D_SERVICE_H
#define SRC_FIELD_D_SERVICE_H

#include "FieldDPlanner.h"

namespace field_d
{
class FieldDService
{
public:
  FieldDService();

private:
  FieldDPlanner planner;
};

}  // namespace field_d
#endif  // SRC_FIELD_D_SERVICE_H
