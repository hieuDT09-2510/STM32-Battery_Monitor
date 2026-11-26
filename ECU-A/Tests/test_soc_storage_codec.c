/*
 * Unit test trên máy host cho bộ mã hóa/giải mã bản ghi SOC 32-byte.
 * Chỉ kiểm chứng định dạng cố định, CRC, phạm vi nghiệp vụ và tràn vòng số thứ tự, không truy cập Flash giả lập hay Flash thật.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_soc_storage.h"

static void TestRecordRoundTrip(void)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE]; /* 32 byte sau khi mã hóa. */
    SocPersistentState_t input;              /* Trạng thái nghiệp vụ trước khi mã hóa. */
    SocPersistentState_t output;             /* Trạng thái nghiệp vụ được khôi phục sau giải mã. */
    uint32_t sequence;                       /* Số thứ tự bản ghi được khôi phục sau giải mã. */

    input.remaining_uAs = 3600000000LL;
    input.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    input.timestamp_ms = 123456U;
    input.soc_centi_percent = 5000U;
    input.learned_capacity_uAh = 1760000U;
    input.soh_valid = 1U;

    SocStorage_EncodeRecord(record, 0x12345678UL, &input);

    assert(record[0] == 0x53U);
    assert(record[1] == 0x4FU);
    assert(record[2] == 0x01U);
    assert(record[26] == 0x00U);
    assert(record[27] == 0x1AU);
    assert(record[28] == 0xDBU);
    assert(record[29] == 0x00U);
    assert(record[30] == 1U);

    assert(SocStorage_DecodeRecord(
               record,
               &sequence,
               &output) == 1U);
    assert(sequence == 0x12345678UL);
    assert(output.remaining_uAs == input.remaining_uAs);
    assert(output.rated_capacity_uAh == input.rated_capacity_uAh);
    assert(output.timestamp_ms == input.timestamp_ms);
    assert(output.soc_centi_percent == input.soc_centi_percent);
    assert(output.learned_capacity_uAh == input.learned_capacity_uAh);
    assert(output.soh_valid == 1U);
}

static void TestCorruptionAndErasedRecordRejected(void)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE]; /* Bản ghi bị cố tình làm hỏng hoặc điền 0xFF. */
    SocPersistentState_t state;              /* Trạng thái nghiệp vụ mã hóa/giải mã. */
    uint32_t sequence;                       /* Số thứ tự đầu ra sau giải mã. */

    state.remaining_uAs = 1000000LL;
    state.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    state.timestamp_ms = 99U;
    state.soc_centi_percent = 1U;
    state.learned_capacity_uAh = 0U;
    state.soh_valid = 0U;

    SocStorage_EncodeRecord(record, 7U, &state);
    /* Đảo 1 bit trong trường dung lượng còn lại, CRC bắt buộc phải từ chối bản ghi này. */
    record[8] ^= 0x01U;
    assert(SocStorage_DecodeRecord(record, &sequence, &state) == 0U);

    (void)memset(record, 0xFF, sizeof(record));
    assert(SocStorage_RecordIsErased(record) == 1U);
    assert(SocStorage_DecodeRecord(record, &sequence, &state) == 0U);
}

static void TestInvalidPhysicalRangeRejected(void)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE];
    SocPersistentState_t state;
    uint32_t sequence;

    /* Ngay cả khi CRC đúng, giá trị nghiệp vụ vượt quá dung lượng đầy định mức cũng bắt buộc phải bị từ chối. */
    state.remaining_uAs = SOC_FULL_CHARGE_UAS + 1LL;
    state.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    state.timestamp_ms = 0U;
    state.soc_centi_percent = 10000U;
    state.learned_capacity_uAh = 0U;
    state.soh_valid = 0U;
    SocStorage_EncodeRecord(record, 1U, &state);
    assert(SocStorage_DecodeRecord(record, &sequence, &state) == 0U);
}

static void TestOldRecordRestoresWithoutSoh(void)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE];
    SocPersistentState_t state;
    uint32_t sequence;

    state.remaining_uAs = SOC_FULL_CHARGE_UAS / 2LL;
    state.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    state.timestamp_ms = 10U;
    state.soc_centi_percent = 5000U;
    state.learned_capacity_uAh = 0U;
    state.soh_valid = 0U;

    SocStorage_EncodeRecord(record, 9U, &state);
    assert(record[26] == 0U);
    assert(record[27] == 0U);
    assert(record[28] == 0U);
    assert(record[29] == 0U);
    assert(record[30] == 0U);
    assert(SocStorage_DecodeRecord(record, &sequence, &state) == 1U);
    assert(sequence == 9U);
    assert(state.soh_valid == 0U);
    assert(state.learned_capacity_uAh == 0U);
}

static void TestSequenceWrapComparison(void)
{
    /* Kiểm chứng tăng tuần tự thông thường, bằng nhau và tràn vòng 32-bit từ 0xFFFFFFFF về 0. */
    assert(SocStorage_IsSequenceNewer(11U, 10U) == 1U);
    assert(SocStorage_IsSequenceNewer(10U, 10U) == 0U);
    assert(SocStorage_IsSequenceNewer(0U, 0xFFFFFFFFUL) == 1U);
    assert(SocStorage_IsSequenceNewer(0xFFFFFFFFUL, 0U) == 0U);
}

int main(void)
{
    TestRecordRoundTrip();
    TestCorruptionAndErasedRecordRejected();
    TestInvalidPhysicalRangeRejected();
    TestOldRecordRestoresWithoutSoh();
    TestSequenceWrapComparison();
    puts("SOC storage codec tests passed");
    return 0;
}
