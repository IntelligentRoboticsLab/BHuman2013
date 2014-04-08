#pragma once

#include "Modules/Modeling/SelfLocator/UKFSample.h"
#include "Tools/SampleSet.h"

#include "Tools/Streams/Streamable.h"

class UKFSamples : public Streamable {
public:
  UKFSamples();

  SampleSet<UKFSample> samples;       /**< Container for all samples. */
private:
  void serialize(In*, Out*);
};
