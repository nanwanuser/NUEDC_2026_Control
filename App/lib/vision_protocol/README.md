# 视觉二进制串口协议

USART1配置为115200、8位数据、1位停止位、无校验。所有多字节整数使用小端序，
坐标使用有符号16位整数表示，单位为0.1 mm。

## 视觉结果帧

```text
AA 55 | version | type | seq | payload_length | payload | crc16 | 0D 0A
 2 B  |   1 B   | 1 B  | 2 B |      2 B       |  N B    |  2 B  |  2 B
```

- `version`：固定为`0x01`。
- `type`：视觉结果为`0x01`。
- `seq`：`uint16_t`循环序号。
- `payload_length`：只包含payload。
- `crc16`：CRC16-CCITT-FALSE，多项式`0x1021`、初值`0xFFFF`，计算范围从
  `version`到payload最后一个字节。
- 结束符：固定为`0x0D 0x0A`。

payload开头为：

```text
decision_mode uint8_t  // 0: fixed ID, 1: general
piece_count   uint8_t  // 1..4
```

之后连续放置`piece_count`块碎片。每块格式为：

```text
id             uint8_t
vertex_count   uint8_t  // 3..5
cx             int16_t
cy             int16_t
vertex[0].x    int16_t
vertex[0].y    int16_t
...
```

一块碎片长度为`6 + 4 * vertex_count`，4块、每块5顶点时完整帧最大118字节。
同一帧内ID不能重复，顶点必须沿轮廓依次排列。

## ACK帧

ACK使用相同的帧格式，`type = 0x80`、payload长度为1，seq与收到的视觉帧相同。
payload状态值：

```text
0: 当前帧有效，继续发送
1: 已连续确认并提交决策，停止发送
2: 数据无效
3: 系统忙
```

## 稳定确认

STM32连续收到3个相近的有效帧才提交决策。模式、碎片数量、ID和顶点数量必须
完全相同，中心和顶点坐标允许相差0.5 mm。比较时按ID匹配碎片，并自动处理顶点
起点循环变化和顺逆时针变化。最终坐标取3帧平均值。

确认后STM32发送状态1的ACK，停止USART1接收、反初始化USART1，并调用
`DecisionTask_Submit()`。

## 采集授权

视觉任务上电后不主动接收，`VisionUart_Arm()`被调用才打开接收器。题目要求先
遮挡摄像头摆放碎片、按键启动时同时移除遮挡，因此启动键之前到达的帧会被丢弃。
决策模式由按键决定，不使用视觉帧里的`decision_mode`字段。

每次`VisionUart_Arm()`会在必要时重新初始化USART1，所以一轮提交后仍可再次武装，
反复测试不需要重新上电。`VisionUart_Abort()`放弃当前采集。

固定ID模式使用前，应用必须调用`VisionUart_SetFixedLayout()`配置目标模板；
通用模式不需要模板。
