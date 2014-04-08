#include "UKFSamples.h"

UKFSamples::UKFSamples() : samples(1) {}

void UKFSamples::serialize(In* in, Out* out) {

  STREAM_REGISTER_BEGIN;
  for ( int i = 0; i < samples.size(); ++i ) {
    STREAM(samples.at(i)); 
  }
  STREAM_REGISTER_FINISH;

}
