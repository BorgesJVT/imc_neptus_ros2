//***************************************************************************
// Copyright 2017 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// Licensed under the Apache License, Version 2.0 (the "License");          *
// you may not use this file except in compliance with the License.         *
// You may obtain a copy of the License at                                  *
//                                                                          *
// http://www.apache.org/licenses/LICENSE-2.0                               *
//                                                                          *
// Unless required by applicable law or agreed to in writing, software      *
// distributed under the License is distributed on an "AS IS" BASIS,        *
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
// See the License for the specific language governing permissions and      *
// limitations under the License.                                           *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 522ff971d12877ebe15aff467ba253d4                            *
//***************************************************************************

#ifndef IMC_DEVCALIBRATIONCONTROL_HPP_INCLUDED_
#define IMC_DEVCALIBRATIONCONTROL_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include <IMC/Base/Config.hpp>
#include <IMC/Base/Message.hpp>
#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/MessageList.hpp>
#include <IMC/Base/JSON.hpp>
#include <IMC/Base/Serialization.hpp>
#include <IMC/Spec/Enumerations.hpp>
#include <IMC/Spec/Bitfields.hpp>

namespace IMC
{
  //! Device Calibration Control.
  class DevCalibrationControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Start.
      DCAL_START = 0,
      //! Stop.
      DCAL_STOP = 1,
      //! Perform Next Calibration Step.
      DCAL_STEP_NEXT = 2,
      //! Perform Previous Calibration Step.
      DCAL_STEP_PREVIOUS = 3
    };

    //! Operation.
    uint8_t op;

    static uint16_t
    getIdStatic(void)
    {
      return 12;
    }

    static DevCalibrationControl*
    cast(Message* msg__)
    {
      return (DevCalibrationControl*)msg__;
    }

    DevCalibrationControl(void)
    {
      m_header.mgid = DevCalibrationControl::getIdStatic();
      clear();
    }

    DevCalibrationControl*
    clone(void) const
    {
      return new DevCalibrationControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DevCalibrationControl& other__ = static_cast<const DevCalibrationControl&>(msg__);
      if (op != other__.op) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DevCalibrationControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DevCalibrationControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
    }
  };
}

#endif
