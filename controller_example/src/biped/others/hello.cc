#include <iostream>
#include <string>

struct point
{
  double x[3];

  const double &get_x() const
  {
    return x[0];
  }

  ~point()
  {
    std::cout << "...";
  }
};

int main()
{
  const double &x = point{1, 2, 4}.get_x();
  // std::cout << &x << "\n";
  std::cout << x << "\n";
  double dddd[] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
  // std::cout << dddd << "\n";
  std::cout << x << "\n";
  return 0;
}
