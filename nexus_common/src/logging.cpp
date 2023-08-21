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

#include "logging.hpp"

namespace nexus::common {

std::string ReportConverter::to_string(const BtLogging::LogReport& report)
{
  std::string indent = "    ";
  std::ostringstream oss;
  oss << "Time elapsed per node:" << std::endl;
  for (const auto& record : report.total_elasped_time)
  {
    oss << indent << record.node_name << "-" << record.node_id << ": " <<
      std::fixed << std::setprecision(3) <<
      std::chrono::duration<double>(record.elapsed_time).count() << "s" <<
      std::endl;
  }
  return oss.str();
}

}
