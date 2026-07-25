#ifndef __SECURITY_H
#define __SECURITY_H

#include "main.h"
#include <stdint.h>
#ifdef __cplusplus
class Security_
{
public:
    // 初始化安全模块：读取UID、检查RDP等级、验证应用程序完整性
    void Init(void);
    
    // 检查当前设备是否已授权（基于UID和密钥）
    bool IsAuthorized(void);
    
    // 使能Flash读保护（Level 1），重启后生效
    void EnableReadProtection(void);
    
    // 禁用Flash读保护（会擦除全部Flash）
    void DisableReadProtection(void);
    
    // 获取当前RDP等级
    uint8_t GetRDPLevel(void);
    
    // 运行时对关键代码段进行解密（需要配合链接脚本使用）
    void DecryptCodeSegment(uint32_t addr, uint32_t size, uint32_t key);
    
    // 系统完整性自检（CRC校验）
    bool SelfCheck(void);

private:
    uint32_t uid[3];            // 96位唯一ID
    uint16_t crc_table[256];    // CRC16表
    uint32_t encryption_key;    // 加密密钥（可从UID派生）
    
    void ReadUID(void);
    void GenerateEncryptionKey(void);
    void InitCRC16Table(void);
    uint16_t CalculateCRC16(uint8_t *buf, uint32_t len);
    bool CheckFlashIntegrity(void);
};
#endif
#endif
/*
在安全方面crc作为一个防止数据错乱的手段。
而非对称加密则是防止篡改：
 针对单片机推荐是ECC工具：
	
后端的程序：
// ========== 【离线一次性执行】生成密钥对，生成后把公钥拷贝到单片机代码 ==========
    public KeyPair generateKeyPair() throws Exception {
        ECParameterSpec ecSpec = ECNamedCurveTable.getParameterSpec(CURVE_NAME);
        KeyPairGenerator g = KeyPairGenerator.getInstance("ECDSA", "BC");
        g.initialize(ecSpec);
        return g.generateKeyPair();
    }

    // 对文件计算SHA256哈希，然后私钥签名，返回Base64签名
    public String signFile(File binFile, PrivateKey privateKey) throws Exception {
        byte[] fileData = Files.readAllBytes(binFile.toPath());

        // 1. SHA256
        MessageDigest digest = MessageDigest.getInstance("SHA-256");
        byte[] hash = digest.digest(fileData);

        // 2. ECDSA签名
        Signature signer = Signature.getInstance("SHA256withECDSA", "BC");
        signer.initSign(privateKey);
        signer.update(hash);
        byte[] signature = signer.sign();

        // Base64方便HTTP文本传输
        return Base64.getEncoder().encodeToString(signature);
    }

    // 公钥可以导出打印，复制到STM32
    public String getPubKeyBase64(PublicKey publicKey) {
        return Base64.getEncoder().encodeToString(publicKey.getEncoded());
    }
在controller层进行分成两个接口，一个是签名验证获取，一个是固件获取
第一个接口在字段里加入version版本，大小，签名

然后提前把服务器写的公钥写入在单片机程序里，写死
然后解析报文时候
手撕实现SHA256_CTX验证算法即可
这样就防止你下载的程序如果被人提前篡改，那么就无法验证通过，也就不会下载到黑客的代码

我这个项目用的是RSA加密，但已经过时了，因为数据太大了
现在使用AES加密
*/