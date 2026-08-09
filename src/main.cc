#include "app_config.h"
#include "common_utils.h"
#include "mmz.h"
#include "supercombo_runtime.h"

#include <nncase/runtime/util.h>

#include <exception>
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;

namespace {

volatile sig_atomic_t g_signal_stop = 0;

} // namespace

void __attribute__((destructor)) cleanup()
{
    nncase::runtime::shrink_memory_pool();
    kd_mpi_mmz_deinit();
}

int main(int argc, char *argv[])
{
    cout << "case " << argv[0] << " built at " << __DATE__ << " " << __TIME__ << endl;

    AppConfig config;
    try {
        config = AppConfig::from_env(argc, argv);
    } catch (const std::exception &e) {
        cout << e.what() << endl;
        return 1;
    }

    install_stop_signal_handlers(&g_signal_stop);

    SupercomboRuntime runtime(config);
    const int ret = config.replay_enabled()
        ? runtime.run_replay()
        : runtime.run_live(&g_signal_stop);

    cout << endl << "exit" << endl;
    return ret;
}
