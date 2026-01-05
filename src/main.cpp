#include "core/application.hpp"
#include <spdlog/spdlog.h>

int main(int argc, char* argv[]) {
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("Stratum v0.1.0");

    stratum::Application app;

    if (!app.init()) {
        spdlog::error("Failed to initialize application");
        return 1;
    }

    app.run();
    app.shutdown();

    return 0;
}
