#ifndef LIBHBC_HBC_ENUMS_HPP
#define LIBHBC_HBC_ENUMS_HPP

namespace hexabus {
  // define first operator as "1" because "0" means "not parsed (yet)".
  enum bool_operator { AND = 1, OR };
  enum comp_operator { STM_COMP_EQ = 1, STM_COMP_LEQ, STM_COMP_GEQ, STM_COMP_LT, STM_COMP_GT, STM_COMP_NEQ };
  enum datatype { DT_UNDEFINED = 0, DT_BOOL = 1, DT_UINT8, DT_UINT32, DT_FLOAT };
  enum access_level { AC_READ = 1, AC_WRITE, AC_BROADCAST };
  enum time_fields { TF_HOUR = 1, TF_MINUTE, TF_SECOND, TF_DAY, TF_MONTH, TF_YEAR, TF_WEEKDAY };
};

#endif // LIBHBC_HBC_ENUMS
