// Part of the MillingStation fork of FluidNC.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "GenericProtocol.h"

namespace Spindles {
    namespace VFD {
        // Delta MS300, configured directly under a `DeltaMS300:` YAML section - no `model:`
        // key needed. The actual command strings/scale factors live in GenericProtocol.cpp's
        // VFDtypes[] table under the name "DeltaMS300"; this class just presets
        // GenericProtocol's _model to that name so afterParse() fills them in automatically.
        // (`ModbusVFD: {model: DeltaMS300, ...}` still works too, for old configs - both paths
        // read the same table entry, so there's only one place the command strings live.)
        class DeltaMS300 : public GenericProtocol {
        public:
            DeltaMS300() : GenericProtocol("DeltaMS300") {}
        };
    }
}
