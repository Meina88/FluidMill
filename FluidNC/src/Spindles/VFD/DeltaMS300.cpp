// Part of the MillingStation fork of FluidNC.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "DeltaMS300.h"

#include "../VFDSpindle.h"

namespace Spindles {
    namespace VFD {
        // Configuration registration
        namespace {
            SpindleFactory::DependentInstanceBuilder<VFDSpindle, DeltaMS300> registration("DeltaMS300");
        }
    }
}
