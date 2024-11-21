# Copyright 2022 Johnson & Johnson
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import json
import re
import os
from jsonschema import validate, ValidationError
import sys
import yaml
from ament_index_python.packages import get_package_share_directory


class NEXUSConfigGenerator:
    """NEXUSConfigGenerator generates Zenoh DDS Bridge configurations."""

    def __init__(
            self, nexus_net_cfg_filepath,
            redf_cfg_filepath, schema_filepath):
        """
        Construct NEXUSConfigGenerator.

        Parameters
        ----------
        nexus_net_cfg_filepath : str
            NEXUS network configuration filepath
        redf_cfg_filepath : str
            REDF configuration filepath
        schema_filepath : str
            NEXUS Network configuration schema filepath

        """
        self.nexus_net_cfg = self.parse_file(nexus_net_cfg_filepath)
        self.redf_cfg = self.parse_file(redf_cfg_filepath)

        self.nexus_net_cfg_schema = self.parse_file(schema_filepath)

        # Set global variables
        self.enable_rest_api = self.nexus_net_cfg["enable_rest_api"]
        self.bridge_mode = self.nexus_net_cfg["mode"]

        self.zenoh_cfg_file_extension = "json5"

    def parse_file(self, filepath):
        """
        Parse a configuration file (YAML/JSON) into a python dictionary.

        Parameters
        ----------
        filepath : str
            filepath to configuration to parse

        Returns
        -------
        dict
            Dictionary parsed from given file

        """
        if filepath is None:
            return {}

        try:
            file_name, file_extension = os.path.splitext(filepath)
            dict_obj = {}
            if file_extension == ".json":
                with open(filepath, "r", encoding="utf-8") as reader:
                    dict_obj = json.load(reader)
            elif file_extension == ".yaml":
                with open(filepath, "rt", encoding="utf-8") as reader:
                    dict_obj = yaml.safe_load(reader.read())
            else:
                print(
                    f"ERROR: File {file_name} is neither of extension '.json' \
                        'nor '.yaml'")
                return None
            return dict_obj
        except Exception as e:
            print(f"Caught exception when parsing file: {filepath}. What: {e}")
            sys.exit(1)

    def validate_nexus_net_cfg_schema(self):
        """
        Check if NEXUS Network configuration follows the pre-defined schema.

        Returns
        -------
        bool
            If true, NEXUS Network configuration follows the schema.

        """
        try:
            validate(self.nexus_net_cfg, schema=self.nexus_net_cfg_schema)
            return True
        except ValidationError as validation_err:
            print("NEXUS Network configuration schema validation failed")
            print(f"Validation error: {validation_err}")

        return False

    def zenoh_cfg(
        self,
        orchestrator,
        allowed_endpoints,
        tcp_connect,
        tcp_listen
    ):
        """
        Configure and return a Zenoh bridge from given parameters.

        Parameters
        ----------
        orchestrator : dict
            Orchestrator dictionary with configuration specific to it
        allowed_endpoints : str
            ROS endpoints that will be allowed through the Zenoh bridge
        tcp_connect : str
            TCP Addresses of Zenoh bridges to connect to
        tcp_listen : str
            TCP Addresses of Zenoh bridges to listen to

        Returns
        -------
        dict
            Python dictionary of Zenoh bridge configuration

        """
        # Prepend "tcp/" to connect and listen endpoints to denote TCP Protocol
        tcp_connect_endpoints = \
            [("tcp/" + addr) for addr in tcp_connect]
        tcp_listen_endpoints = \
            [("tcp/" + addr) for addr in tcp_listen]

        zenoh_dict = {
            "plugins": {
                "ros2dds": {
                    "domain": orchestrator["domain_id"],
                    "namespace": orchestrator["namespace"],
                    "allow": orchestrator["allow"],
                    "queries_timeout": orchestrator["queries_timeout"],
                },
            },
            "mode": self.bridge_mode,
            "connect": {"endpoints": tcp_connect_endpoints},
            "listen": {"endpoints": tcp_listen_endpoints},
        }

        if self.enable_rest_api:
            zenoh_dict["plugins"]["rest"] = {
                "http_port": orchestrator["rest_api_http_port"]
            }

        return zenoh_dict

    def generate_zenoh_config(self, output_dir):
        """
        Generate Zenoh bridge configs and output to directory 'output_dir'.

        Parameters
        ----------
        output_dir : str
            Output directory for Zenoh configurations

        """
        nexus_net_cfg = self.nexus_net_cfg

        orchestrators_zenoh_cfg = []

        allowed_endpoints = self.generate_allowed_endpoints(
            self.redf_cfg)

        # Generate system orchestrator zenoh dictionary
        for orchestrator in nexus_net_cfg["system_orchestrators"]:

            zenoh_cfg = self.zenoh_cfg(
                orchestrator=orchestrator,
                allowed_endpoints=allowed_endpoints,
                tcp_connect=[],
                tcp_listen=orchestrator["tcp_listen"]
            )

            orchestrators_zenoh_cfg.append(zenoh_cfg)

        # Generate workcell orchestrators zenoh dictionary
        for orchestrator in nexus_net_cfg["workcell_orchestrators"]:

            zenoh_cfg = self.zenoh_cfg(
                orchestrator=orchestrator,
                allowed_endpoints=allowed_endpoints,
                tcp_connect=orchestrator["tcp_connect"],
                tcp_listen=[]
            )

            orchestrators_zenoh_cfg.append(zenoh_cfg)

        # Write system orchestrator config to file
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        for zenoh_cfg in orchestrators_zenoh_cfg:
            write_filepath = os.path.join(
                output_dir,
                zenoh_cfg["plugins"]["ros2dds"]["namespace"]
                + "."
                + self.zenoh_cfg_file_extension,
            )
            del zenoh_cfg["plugins"]["ros2dds"]["namespace"]
            self.write_to_json(write_filepath, zenoh_cfg)
            print(f"Generated Zenoh configuration at {write_filepath}")

    def generate_allowed_endpoints(self,
                                   redf_cfg
                                   ):
        """
        Generate allowed endpoints string from REDF configuration.

        This string is used to only allow selected ROS endpoints through the
        Zenoh bridges.

        Parameters
        ----------
        redf_cfg : dict
            REDF Configuration dictionary

        Returns
        -------
        str
            Allowed endpoints string

        """
        allowed_endpoints = []

        for endpoint in redf_cfg.get('endpoints', []):
            if endpoint.get("topic"):
                allowed_endpoints.append(endpoint["topic"])
            elif endpoint.get("action_name"):
                allowed_endpoints.append(endpoint["action_name"])
            elif endpoint.get("service_name"):
                allowed_endpoints.append(endpoint["service_name"])

        allowed_endpoints_str = ""
        for endpoint_str in allowed_endpoints:
            # Searches for namespace defined in curly braces in REDF endpoints.
            namespace_search = re.compile(r'\{\w*\}')
            namespace_found = namespace_search.search(endpoint_str)
            if namespace_found:
                endpoint_str = namespace_search.sub(".*", endpoint_str)
            allowed_endpoints_str += endpoint_str + "|"

        # Remove the last "|" character
        allowed_endpoints_str = allowed_endpoints_str[:-1]

        return allowed_endpoints_str

    def write_to_json(self, filepath, cfg_dict):
        """
        Write python dictionary to to a JSON File.

        Parameters
        ----------
        filepath : str
            Filepath to write to
        cfg_dict : dict
            Zenoh configuration

        """
        with open(filepath, "wt", encoding="utf-8") as writer:
            json.dump(cfg_dict, writer, indent=2, default=str)


def main(argv=sys.argv):
    """Entrypoint."""

    parser = argparse.ArgumentParser(
        description="Generate Zenoh configurations from a NEXUS Network \
            and REDF Configuration",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument(
        '-n',
        '--nexus_net_cfg',
        required=True,
        type=str,
        help="The NEXUS Network configuration filepath")

    parser.add_argument(
        '-r',
        '--redf_cfg',
        required=False,
        type=str,
        help="The REDF configuration filepath")

    parser.add_argument(
        "-o",
        "--output",
        required=True,
        type=str,
        help="Output directory for Zenoh bridge configurations",
    )

    args = parser.parse_args(argv[1:])

    # Get REDF yaml filepath
    redf_cfg_filepath = args.redf_cfg
    # Get Network configuration yaml filepath
    nexus_net_cfg_filepath = args.nexus_net_cfg
    # Get schema filepath
    schema_filepath = os.path.join(
        get_package_share_directory("nexus_network_configuration"),
        "schemas",
        "nexus_network_schema.json"
    )

    nexus_config_gen = NEXUSConfigGenerator(
        nexus_net_cfg_filepath, redf_cfg_filepath, schema_filepath
    )

    if not nexus_config_gen.validate_nexus_net_cfg_schema():
        print(
            "NEXUS Network configuration does not follow the schema. Please \
                check again before generating the Zenoh configs"
        )
        return

    nexus_config_gen.generate_zenoh_config(args.output)


if __name__ == "__main__":
    main(sys.argv)
