#include <iostream>
#include <cstring>
#include <optional>

void stop(std::optional<int> param = std::nullopt) {
    std::cout << "__func__ = '" << __func__ << "'" << std::endl;
    
    bool is_stop = (strcmp(__func__, "stop") == 0);
    std::cout << "strcmp result: " << is_stop << std::endl;
}

int main() {
    stop();
    stop(42);
    return 0;
}
