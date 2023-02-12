# Alias-2023

## 走行マニュアル

+ ライントレースの試走行

    + ロータリースイッチの状態を F にする。 ←操作
    + 主電源を ON にする。 ←操作
    + LED が 赤色・緑色・青色に点滅することを確認する。
    + LED が 黄色に３回点滅することを確認する。
    + ラインセンサのキャリブレーションを行う。 ←操作
        > LED が赤色に変化しなくなったら終了。
    + ロータリースイッチの状態を 5 にする。 ←操作
    + LED が 緑色・黄色に３回点滅することを確認する。
    + 走行開始。
    + 走行終了。
    + LED が 黄色に点灯することを確認する。

## ロータリースイッチの状態

+ ロータリースイッチの状態を変化させると、
「現在の状態」 が 「変化させた状態」 に変更されるまでに３秒間待機する必要がある。

    + B : DEBUG_MODE
        + LED を赤色に３回点滅させる
        + g_mode = STANDBY
        + ３秒間待機する
        + LED を赤色に３回点滅させる
        + FlashTest を実行
        + 正常終了なら LED を虹色に光らせ続ける
    + C : DEBUG_MODE
        + LED を緑色・青色に３回点滅させる
        + g_mode = VELOCITY_CONTROL_DEBUG
    + D : DEBUG_MODE
        + LED を緑色・黄色に３回点滅させる
        + g_mode = LINE_TRACE_DEBUG
    + E : DEBUG_MODE
        + LED を緑色に３回点滅させる
        + g_mode = INITIAL_DEBUG
    + E
        + LED を赤色に３回点滅させる
        + g_mode = STANDBY
        + ３秒間待機する
        + LED を赤色に３回点滅させる
        + FLASH_DATA を全て消去する
        + 消去が正常終了なら LED を虹色に光らせ続ける
    + F
        + LED を黄色に３回点滅させる
        + g_mode = READY
    + 0
        + g_mode = STANDBY
        + LED を虹色に光らせ続ける
    + 1
        + LED を青色に３回点滅させる
        + g_mode = FIRST_RUN
    + 2 
        + LED を紫色に３回点滅させる
        + g_mode = SECOND_RUN
    + 4
        + LED を緑色・青色に３回点滅させる
        + g_mode = VELOCITY_CONTROL
    + 5
        + LED を緑色・黄色に３回点滅させる
        + g_mode = LINE_TRACE
    + その他
        + g_mode = STANDBY
        + LED を白色に点滅させ続ける
