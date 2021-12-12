#include "pch.h"
#include "ppl.h"
#include <iostream>

#include <iostream>
#include"amp.h"
#include<array>
#include<iostream>

using namespace concurrency;

std::vector<accelerator> findAccelerators() {
	std::vector<accelerator> accels;
	accels = accelerator::get_all();

	for (int i = 0; i < accels.size(); i++) {
		std::wcout << i + 1 << "th device = " << accels[i].get_description() << ", " << (accels[i].get_is_emulated() ? "enum" : "normal") << std::endl;
	}

	accels.erase(std::remove_if(accels.begin(), accels.end(), [](accelerator& accel) {return accel.get_is_emulated(); }), accels.end());

	return accels;
}

int main()
{
	constexpr int dim = 1;
	const int size = 100'000'000;
	int* arr = new int[size];

#if 0
	parallel_for
	Concurrency::parallel_for(0, count, 1, [&](int i)
		{

		});

#else

	std::vector<accelerator> accels = findAccelerators();
	auto it = std::max_element(accels.begin(), accels.end(), [](const concurrency::accelerator& rhs, const concurrency::accelerator& lhs)
			{
				return rhs.get_dedicated_memory() < lhs.get_dedicated_memory();
			});
	concurrency::accelerator accel = *it;
	concurrency::extent<dim> ex;
	ex[0] = size;

	concurrency::array_view<int, dim> view(size, &arr[0]);
	parallel_for_each(accel.get_default_view(),
		ex,
		[=](concurrency::index<dim> gindex) restrict(amp) {
			view[gindex] = 333;
		}
	);

	view.synchronize();
#endif

	for (int i = 0; i < size; i++) {
		std::cout << arr[i] << ",";
	}
	std::cout << std::endl;

	return 0;
}