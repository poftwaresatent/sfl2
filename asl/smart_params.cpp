#include "smart_params.hpp"
#include <sfl/util/strutil.hpp>
#include <sfl/util/OptionDictionary.hpp>

using namespace sfl;
using namespace boost;

smartparams::
smartparams(shared_ptr<sfl::OptionDictionary> opt)
  : expo_parameters(opt),
    model_axlewidth(0.8),
    model_phi_max(1.0),
    model_phid_max(3.0)
{
  string_to(opt->GetOption("model_axlewidth"), model_axlewidth);
  string_to(opt->GetOption("model_phi_max"), model_phi_max);
  string_to(opt->GetOption("model_phid_max"), model_phid_max);
}
