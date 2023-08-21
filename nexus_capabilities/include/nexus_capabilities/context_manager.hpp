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

#ifndef NEXUS_CAPABILITIES__CONTEXT_MANAGER_HPP
#define NEXUS_CAPABILITIES__CONTEXT_MANAGER_HPP

#include "context.hpp"

namespace nexus {

//==============================================================================
// TODO(YV): Template this class with context so that it can be reused
// for system and workcell orchestrators.
class ContextManager
{
public:
  std::shared_ptr<Context> current_context() const
  {
    return this->_cur_ctx;
  }

  void set_active_context(const std::shared_ptr<Context>& ctx)
  {
    this->_cur_ctx = ctx;
  }

private:
  std::shared_ptr<Context> _cur_ctx;
};

} // namespace nexus

#endif // NEXUS_CAPABILITIES__CONTEXT_MANAGER_HPP
