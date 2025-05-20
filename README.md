项目使用的是微雪esp32-s3-touch-lcd-2,带8m的psram，16m的flash，摄像头使用的是ov5640，更多详细信息可以去微雪的官网查看。

芯片运行基于esp-idf，运行tensorflow lite使用的是官方的依赖ESP-TFLITE-MICRO，目前网上的教程比较少，我会介绍一下关键的步骤和大致流程。

在tensorflow里，这里使用的是vgg16经典模型





