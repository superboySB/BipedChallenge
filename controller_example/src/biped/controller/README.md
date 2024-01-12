# Controllers

这里的控制器(controller)特指力控制器。它也被称为whole body controller或inverse dynamics.

类的名字和文件名是对应的，不需要额外解释。

`FeedbackController`是ALIP行走规划器对应的力控制器，它总是单腿支撑。

`TauController`是（正在）扩展到双腿支撑和腾空的版本，它还统一虚约束的结构。我没有想到很好的名字，
`FeedbackController`是不错的名字，可惜被用掉了。`PFL`也许是个不错的想法。
我希望在名字中指出它“虚约束”“反馈线性化”之类，可是它很容易太长了。

`PivotModelController`是一个想法，它将单脚支撑的情况使用定基模型计算，把支撑脚看成pivot，
roll方向可动，希望能减少计算开销时间。
