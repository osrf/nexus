/*
 * Copyright (C) 2022 Johnson & Johnson
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef NEXUS_WORKCELL_ORCHESTRATOR__EXECPTIONS_HPP
#define NEXUS_WORKCELL_ORCHESTRATOR__EXECPTIONS_HPP

#include <stdexcept>
#include <string>

namespace nexus::workcell_orchestrator {

class DiscoveryError : public std::runtime_error
{
public: DiscoveryError(const std::string& msg)
  : std::runtime_error(msg) {}
};

class RegistrationError : public std::runtime_error
{
public: int32_t error_code;

  /**
   * Create new instance with message and error code.
   * @param msg error message
   * @param error_code error code as described in RegisterWorkcell or RegisterTransporter
  */
public: RegistrationError(const std::string& msg, int32_t error_code)
  : std::runtime_error(msg), error_code(error_code) {}
};

}

#endif
