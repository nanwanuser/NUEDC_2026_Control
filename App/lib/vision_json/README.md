# 视觉 JSON 解析

该模块只负责把一个完整 JSON 消息解析成 `DecisionVisionFrame`，不负责 UART
收包或 JSON 分帧。解析过程不使用动态内存。

## 数据格式

```json
{
  "type": "VISION_RESULT",
  "seq": 12,
  "pieces": [
    {
      "id": 0,
      "cx_mm": 83.4,
      "cy_mm": 67.2,
      "vertex_count": 4,
      "vertices_mm": [
        [72.1, 58.0],
        [95.6, 58.2],
        [101.3, 74.1],
        [80.4, 80.6]
      ]
    }
  ]
}
```

约束如下：

- `type` 必须为 `VISION_RESULT`。
- `seq` 为 `uint32_t` 非负整数。
- `pieces` 数量为 1 至 4。
- `id` 范围为 0 至 255，同一消息内不能重复。
- `vertex_count` 为 3 至 5，且必须与 `vertices_mm` 数量一致。
- 坐标绝对值不超过 1000 mm，必须是有限数值。
- 顶点需要沿轮廓依次排列；几何合法性由后续决策模块继续检查。
- 未识别的扩展字段会被忽略。

## 调用

```c
DecisionVisionFrame frame;
VisionJsonResult result;
DecisionTaskRequest request;

DecisionTask_GetDefaultRequest(&request);
result = VisionJson_Parse(rx_buffer, json_length, &frame);
if (result == VISION_JSON_RESULT_OK) {
    request.vision = frame;
    DecisionTask_Submit(&request);
}
```

`json_length` 不包含字符串末尾的 `\0`。已有以 `\0` 结尾的字符串可以使用
`VisionJson_ParseString()`。

解析器内部使用固定 token 缓冲区，因此不能被多个任务同时调用。后续 UART 接收任务应当
作为唯一调用者。
