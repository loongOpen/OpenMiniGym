#include <dlfcn.h>
#include <iostream>
#include "armdecode_R.h" // 您的头文件


// extern "C"{


double* ik_right(double real_[7]) {
    // 加载动态链接库
    void* handle = dlopen("/home/tinymal/下载/aloha/act_plus3/DrSun_IK_solver/ttst/ik_7dof_fourier/libarmdecode_v5_R.so", RTLD_LAZY);
    if (!handle) {
        // 无法加载库
        std::cerr << "Cannot load library: " << dlerror() << '\n';
        return NULL;
    }

    // 获取函数指针
typedef void (*ik_7dof_fourier_R_type)(double, double, double, double, double, double, double, std::vector<float>&);
ik_7dof_fourier_R_type ik_7dof_fourier_R = (ik_7dof_fourier_R_type)dlsym(handle, "_Z17ik_7dof_fourier_RdddddddRSt6vectorIfSaIfEE");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
        // 无法获取函数
        std::cerr << "Cannot load symbol 'ik_7dof_fourier_R': " << dlsym_error << '\n';
        dlclose(handle);
        return NULL;
    }
    double* thetac = new double[7];
    double main1,main2,main3,main4,main5,main6,main7;
    main1 = real_[0];
    main2 = real_[1];
    main3 = real_[2];
    main4 = real_[3];
    main5 = real_[4];
    main6 = real_[5];
    main7 = real_[6];
    std::vector<float> theta(7);
    // std::couint main()t<<"111111"<<std::endl;
    // 现在可以调用函数
    ik_7dof_fourier_R(main1, main2, main3, main4, main5, main6, main7, theta);
    for (size_t i = 0; i < 7; i++)
    {
        thetac[i]=theta[i];
    }
    // std::cout<<std::endl;
    
    for (size_t i = 0; i < 7; i++)
    {
        std::cout<<thetac[i]<<" ";
    }
    // std::cout<<std::endl;
    // 关闭库
    dlclose(handle);
    return thetac;
}

    void free_memory(double* ptr) {
        delete[] ptr;
    }

int main()
{
    double mam[7];
    mam[0] = 1.8507;
    mam[1] = 0.2145;
    mam[2] = -0.0262;
    mam[3] = -243.1;
    mam[4] = 285.3;
    mam[5] = 148.8;
    mam[6] = -0.53;
    ik_right(mam);
    return 0;
}
// }

