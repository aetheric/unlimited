trait Point<T> {
	//
}

struct Point2<T> {

	pub x: T,

	pub y: T

}

impl<T> Point<T> for Point2<T> {
	//
}

struct Point3<T> {

	pub x: T,

	pub y: T,

	pub z: T

}

impl<T> Point<T> for Point3<T> {
	//
}
