#include "plugin.h"
#include "Encoder.h"
#include <cedar/processing/ElementDeclaration.h>

void pluginDeclaration(cedar::aux::PluginDeclarationListPtr plugin)
{
    cedar::proc::ElementDeclarationPtr summation_decl
    (
        new cedar::proc::ElementDeclarationTemplate <Encoder>("Utilities")
    );
    plugin->add(summation_decl);
}
