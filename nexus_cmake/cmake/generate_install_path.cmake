# Generates a header file containing the install path of a package.
# Usage: generate_install_path(<FILE>)
function(generate_install_path)
  configure_file(${NEXUS_CMAKE_DIR}/install_path.hpp.in ${ARGV0})
endfunction()
