import struct
import zlib
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization

# 加载RSA私钥
with open("priv.key", "rb") as f:
    priv_key = serialization.load_pem_private_key(f.read(), password=None)

# 原始APP bin
with open("app.bin", "rb") as f:
    app_bin = f.read()

# 计算CRC32 (和单片机软件CRC32-IEEE保持一致，注意翻转)
crc_raw = zlib.crc32(app_bin)
crc32_val = crc_raw & 0xFFFFFFFF

# SHA256哈希
digest = hashes.Hash(hashes.SHA256())
digest.update(app_bin)
hash_data = digest.finalize()

# RSA-PKCS1v15 签名
sign_data = priv_key.sign(
    hash_data,
    padding.PKCS1v15(),
    hashes.SHA256()
)

# 组装头部 FirmwareHead_t
head_buf = struct.pack("<II", len(app_bin), crc32_val)
head_buf += sign_data

# 输出最终升级包：头部 + APP固件
upgrade_bin = head_buf + app_bin
with open("upgrade.bin", "wb") as f:
    f.write(upgrade_bin)
print("升级包生成完成")
