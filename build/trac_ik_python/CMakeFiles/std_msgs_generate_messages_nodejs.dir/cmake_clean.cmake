file(REMOVE_RECURSE
  "trac_ik_wrapPYTHON_wrap.c"
  "trac_ik_wrap.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/std_msgs_generate_messages_nodejs.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
