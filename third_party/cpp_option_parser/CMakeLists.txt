add_library (option_parser optionParser.cpp optionParser.hpp)

set_target_properties(option_parser PROPERTIES CXX_STANDARD 11)
set_target_properties(option_parser PROPERTIES CXX_STANDARD_REQUIRED ON)
