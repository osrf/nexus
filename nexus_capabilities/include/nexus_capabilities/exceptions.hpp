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

#ifndef NEXUS_CAPABILITIES__EXCEPTIONS_HPP
#define NEXUS_CAPABILITIES__EXCEPTIONS_HPP

#include <stdexcept>
#include <string>

namespace nexus {

/**
 * Base class for all errors from capabilities.
 */
class CapabilityError : public std::runtime_error
{
public: CapabilityError(const std::string& msg)
  : std::runtime_error(msg) {}
};

}

#endif // NEXUS_CAPABILITIES__EXCEPTIONS_HPP
