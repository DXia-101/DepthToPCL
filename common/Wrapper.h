#pragma once
#include <iostream>

namespace te {
	class Storable {
	public:
		virtual ~Storable() {}
		virtual void serialize(std::ostream&) const = 0;
		virtual void deserialize(std::istream&) = 0;
	};

	template<typename T>
	class Wrapper : public Storable {
		T data;

		Wrapper(T d) : data(d) {}

		void serialize(std::ostream& os) const override {
			os.write(reinterpret_cast<const char*>(&data), sizeof(data));
		}

		void deserialize(std::istream& is) override {
			is.read(reinterpret_cast<char*>(&data), sizeof(data));
		}
	};
}