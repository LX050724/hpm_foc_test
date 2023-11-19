local p_justFloat = Proto("justFloat", "CDC Vofa+ justFloat")
local f_data = ProtoField.float("justFloat.data", "Data", base.FLOAT)
local f_end = ProtoField.uint32("justFloat.end", "End")

local data_dis = Dissector.get("data")

p_justFloat.fields = {f_data, f_end}

local function justFloat_dissector(buf, pkt, root)
    local buf_len = buf:len();
    if buf_len < 4 or (buf_len % 4 ~= 0) then
        return false
    end

    local t = root:add(p_justFloat, buf)

    for i = 0, buf_len - 4, 4 do
        d = buf(i, 4)
        if d:le_uint() == 0x7f800000 then
            t:add(f_end, d:le_uint())
        else
            t:add(f_data, d:le_float())
        end
    end

    return true
end

function p_justFloat.dissector(buf, pkt, root)
    if not justFloat_dissector(buf, pkt, root) then
        data_dis:call(buf, pkt, root)
    end
end

local usb_dst_table = DissectorTable.get("usb.bulk")
print("usb_dst_table type", type(usb_dst_table))
usb_dst_table:add(0x0a, p_justFloat)
