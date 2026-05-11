# Vision UART Parser Test Reference

## Protocol Format

```
$SHAPE,TARGET,VALID\r\n
```

## Valid Frames (ACCEPTED)

| Frame | shape | target | valid |
|---|---|---|---|
| `$CIRCLE,A,1\r\n` | SHAPE_CIRCLE | TARGET_A | true |
| `$TRIANGLE,B,1\r\n` | SHAPE_TRIANGLE | TARGET_B | true |
| `$RECT,C,1\r\n` | SHAPE_RECT | TARGET_C | true |
| `$PENTAGON,D,1\r\n` | SHAPE_PENTAGON | TARGET_D | true |
| `$NONE,X,0\r\n` | SHAPE_NONE | TARGET_NONE | false |

All five frames set `has_result = true`.

## Invalid Frames (REJECTED, has_result unchanged)

| Frame | Reject Reason |
|---|---|
| `$CIRCLE,B,1\r\n` | target B does not match CIRCLE (expected A) |
| `$RECT,X,1\r\n` | target X does not match RECT (expected C) |
| `$NONE,A,0\r\n` | target A does not match NONE (expected X) |
| `$TRIANGLE,B,0\r\n` | valid=0 does not match TRIANGLE (expected 1) |
| `$CIRCLE,A,0\r\n` | valid=0 does not match CIRCLE (expected 1) |
| `$NONE,X,1\r\n` | valid=1 does not match NONE (expected 0) |
| `$FOO,A,1\r\n` | unknown shape name |
| `$CIRCLE,,1\r\n` | empty target field |
| `$CIRCLE,A,2\r\n` | valid field not '0' or '1' |
| `$CIRCLE,AB,1\r\n` | target field has more than one char |
| `CIRCLE,A,1\r\n` | missing '$' start marker (never enters collect) |

## Edge Cases

| Input | Behavior |
|---|---|
| 41+ bytes after '$' without '\n' | Buffer overflow, frame discarded, reset to wait '$' |
| `$CIR$CIRCLE,A,1\r\n` | Mid-frame '$' restarts collection; second frame parsed OK |
| `$CIRCLE,A,1\n` | Accepted (bare '\n' without '\r' is valid) |
| `$CIRCLE,A,1\r\r\n` | Accepted ('\r' characters are silently skipped) |
| Empty line `$\r\n` | Rejected (not 3 fields) |

## Debug Integration

To test without real UART hardware, feed bytes manually from the debug UART
or a test loop:

```c
const char *frame = "$CIRCLE,A,1\r\n";
VisionResult_t result;
while (*frame) {
    vision_uart_feed_byte((uint8_t)*frame++);
}
/* now vision_uart_take_result(&result) should return true */
```
