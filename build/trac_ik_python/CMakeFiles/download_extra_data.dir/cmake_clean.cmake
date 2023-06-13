file(REMOVE_RECURSE
  "trac_ik_wrapPYTHON_wrap.c"
  "trac_ik_wrap.py"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
