/*
 * Copyright (C) 2023 Johnson & Johnson
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

#ifndef NEXUS_COMMON__ERROR_HPP
#define NEXUS_COMMON__ERROR_HPP

#include <exception>
#include <memory>
#include <type_traits>
#include <variant>

namespace nexus::common {

/**
 * Represents the result of a function call that may return an error.
 *
 * Example usage:
 * ```cpp
 * const auto& result = foo();
 * if (result.error()) {
 *   // handle error
 * }
 * // use `result.value()` to get pointer to result value.
 * ```
 */
template<typename T, typename E = std::exception>
class Result
{
public: Result(T t)
  : _var(std::move(t)) {}

public: Result(std::shared_ptr<E> e)
  : _var(std::move(e)) {}

public:
  template<typename DE, typename = std::enable_if_t<std::is_base_of_v<E, DE>>>
  Result(DE de)
  : _var(std::make_shared<DE>(std::move(de))) {}

public: T* value()
  {
    return std::get_if<T>(this->_var);
  }

public: const T* value() const
  {
    return std::get_if<T>(this->_var);
  }

public: E* error()
  {
    return std::get_if<std::shared_ptr<E>>(this->_var).data();
  }

public: const E* error() const
  {
    return std::get_if<std::shared_ptr<E>>(this->_var).data();
  }

private: std::variant<T, std::shared_ptr<E>> _var;
};

}

#endif
