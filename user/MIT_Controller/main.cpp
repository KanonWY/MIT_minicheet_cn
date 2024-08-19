#include "MIT_Controller.hpp"
#include <main_helper.h>

/**
 * @brief 运控程序入口
 */
int main(int argc, char** argv)
{
    main_helper(argc, argv, new MIT_Controller());
    return 0;
}
